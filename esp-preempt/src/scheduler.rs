use core::{ffi::c_void, ptr::NonNull};

use allocator_api2::boxed::Box;
use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::semaphore::{SemaphoreImplementation, SemaphorePtr};
use esp_sync::NonReentrantMutex;

use crate::{
    InternalMemory,
    TICK_RATE,
    run_queue::RunQueue,
    semaphore::Semaphore,
    task::{self, Context, TaskAllocListElement, TaskDeleteListElement, TaskList, TaskPtr},
    timer::TimeDriver,
    timer_queue,
};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// Due to how our interrupts work, we may have a timer event and a yield at the same time. Their
// order of processing is an implementation detail in esp-hal. We need to be able to store multiple
// events to not miss any.
pub(crate) struct SchedulerEvent(usize);

impl SchedulerEvent {
    // If this is NOT set, the event is a cooperative yield of some sort (time slicing, self-yield).
    const TASK_BLOCKED: usize = 1 << 0;

    // If this is set, the timer queue should be processed.
    const TIMER_EVENT: usize = 1 << 1;

    fn contains(self, bit: usize) -> bool {
        self.0 & bit != 0
    }
    fn set(&mut self, bit: usize) {
        self.0 |= bit;
    }

    #[expect(dead_code)]
    pub(crate) fn set_blocked(&mut self) {
        self.set(Self::TASK_BLOCKED)
    }

    pub(crate) fn set_timer_event(&mut self) {
        self.set(Self::TIMER_EVENT)
    }

    pub(crate) fn is_cooperative_yield(self) -> bool {
        !self.contains(Self::TASK_BLOCKED)
    }

    pub(crate) fn is_timer_event(self) -> bool {
        self.contains(Self::TIMER_EVENT)
    }
}

pub(crate) struct SchedulerState {
    /// Pointer to the current task.
    pub(crate) current_task: Option<TaskPtr>,

    /// A list of all allocated tasks
    pub(crate) all_tasks: TaskList<TaskAllocListElement>,

    /// A list of tasks ready to run
    pub(crate) run_queue: RunQueue,

    /// Pointer to the task that is scheduled for deletion.
    pub(crate) to_delete: TaskList<TaskDeleteListElement>,

    pub(crate) time_driver: Option<TimeDriver>,

    pub(crate) event: SchedulerEvent,
}

unsafe impl Send for SchedulerState {}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            current_task: None,
            all_tasks: TaskList::new(),
            run_queue: RunQueue::new(),
            to_delete: TaskList::new(),

            time_driver: None,
            event: SchedulerEvent(0),
        }
    }

    pub(crate) fn set_time_driver(&mut self, driver: TimeDriver) {
        self.time_driver = Some(driver);
    }

    fn delete_marked_tasks(&mut self) {
        while let Some(to_delete) = self.to_delete.pop() {
            self.all_tasks.remove(to_delete);
            self.run_queue.remove(to_delete);
            unwrap!(self.time_driver.as_mut())
                .timer_queue
                .remove(to_delete);

            unsafe {
                let task = Box::from_raw_in(to_delete.as_ptr(), InternalMemory);
                core::mem::drop(task);
            }
        }
    }

    fn select_next_task(&mut self, currently_active_task: TaskPtr) -> Option<TaskPtr> {
        // At least one task must be ready to run. If there are none, we can't do anything - we
        // can't just WFI from an interrupt handler. We should create an idle task that WFIs for us,
        // and can be replaced with auto light sleep.
        let next = unwrap!(self.run_queue.pop(), "There are no tasks ready to run.");

        if next == currently_active_task {
            return None;
        }

        Some(next)
    }

    fn run_scheduler(&mut self, task_switch: impl FnOnce(TaskPtr, TaskPtr)) {
        self.delete_marked_tasks();
        let current_task = unwrap!(self.current_task);

        let event = core::mem::take(&mut self.event);
        if event.is_cooperative_yield() {
            // Current task is still ready, mark it as such.
            self.run_queue.mark_same_priority_task_ready(current_task);
        }

        if event.is_timer_event() {
            unwrap!(self.time_driver.as_mut()).handle_alarm(|ready_task| {
                self.run_queue.mark_task_ready(NonNull::from(ready_task));
            });
        }

        if let Some(next_task) = self.select_next_task(current_task) {
            task_switch(current_task, next_task);
            self.current_task = Some(next_task);
        }

        self.arm_time_slice_alarm();
    }

    #[cfg(xtensa)]
    pub(crate) fn switch_task(&mut self, trap_frame: &mut esp_hal::trapframe::TrapFrame) {
        self.run_scheduler(|mut current_task, mut next_task| {
            task::save_task_context(unsafe { current_task.as_mut() }, trap_frame);
            task::restore_task_context(unsafe { next_task.as_mut() }, trap_frame);
        });
    }

    #[cfg(riscv)]
    pub(crate) fn switch_task(&mut self) {
        self.run_scheduler(|mut current_task, mut next_task| {
            let old_ctx = unsafe { &raw mut current_task.as_mut().trap_frame };
            let new_ctx = unsafe { &raw mut next_task.as_mut().trap_frame };

            crate::task::arch_specific::task_switch(old_ctx, new_ctx);
        });
    }

    fn arm_time_slice_alarm(&mut self) {
        let ready_tasks = !self.run_queue.is_empty();
        unwrap!(self.time_driver.as_mut()).arm_next_wakeup(ready_tasks);
    }

    pub(crate) fn schedule_task_deletion(&mut self, task_to_delete: *mut Context) -> bool {
        let current_task = unwrap!(self.current_task);
        let task_to_delete = NonNull::new(task_to_delete).unwrap_or(current_task);
        let is_current = task_to_delete == current_task;

        self.to_delete.push(task_to_delete);

        is_current
    }
}

pub(crate) struct Scheduler {
    inner: NonReentrantMutex<SchedulerState>,
}

impl Scheduler {
    pub(crate) fn with<R>(&self, cb: impl FnOnce(&mut SchedulerState) -> R) -> R {
        self.inner.with(cb)
    }

    pub(crate) fn yield_task(&self) {
        task::yield_task();
    }

    pub(crate) fn current_task(&self) -> TaskPtr {
        task::current_task()
    }
}

esp_radio_preempt_driver::scheduler_impl!(pub(crate) static SCHEDULER: Scheduler = Scheduler {
    inner: NonReentrantMutex::new(SchedulerState::new())
});

impl esp_radio_preempt_driver::Scheduler for Scheduler {
    fn initialized(&self) -> bool {
        self.with(|scheduler| scheduler.time_driver.is_some())
    }

    fn enable(&self) {
        // allocate the main task
        task::allocate_main_task();
        task::setup_multitasking();
        timer_queue::create_timer_task();

        self.with(|scheduler| unwrap!(scheduler.time_driver.as_mut()).start());
    }

    fn disable(&self) {
        self.with(|scheduler| unwrap!(scheduler.time_driver.as_mut()).stop());
        task::disable_multitasking();
        task::delete_all_tasks();
    }

    fn yield_task(&self) {
        self.yield_task();
    }

    fn yield_task_from_isr(&self) {
        self.yield_task();
    }

    fn max_task_priority(&self) -> u32 {
        RunQueue::PRIORITY_LEVELS as u32 - 1
    }

    fn task_create(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        _priority: u32,
        _pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> *mut c_void {
        let task = Box::new_in(Context::new(task, param, task_stack_size), InternalMemory);
        let task_ptr = NonNull::from(Box::leak(task));

        SCHEDULER.with(|state| {
            state.all_tasks.push(task_ptr);
            state.run_queue.mark_task_ready(task_ptr);
        });

        task_ptr.as_ptr().cast()
    }

    fn current_task(&self) -> *mut c_void {
        self.current_task().as_ptr().cast()
    }

    fn schedule_task_deletion(&self, task_handle: *mut c_void) {
        task::schedule_task_deletion(task_handle as *mut Context)
    }

    fn current_task_thread_semaphore(&self) -> SemaphorePtr {
        task::with_current_task(|task| {
            if task.thread_semaphore.is_none() {
                task.thread_semaphore = Some(Semaphore::create(1, 0));
            }

            unwrap!(task.thread_semaphore)
        })
    }

    fn usleep(&self, us: u32) {
        trace!("usleep");
        unsafe extern "C" {
            fn esp_rom_delay_us(us: u32);
        }

        const MIN_YIELD_TIME: u32 = 1_000_000 / TICK_RATE;
        if us < MIN_YIELD_TIME {
            // Short wait, just sleep
            unsafe { esp_rom_delay_us(us) };
        } else {
            const MIN_YIELD_DURATION: Duration = Duration::from_micros(MIN_YIELD_TIME as u64);
            let sleep_for = Duration::from_micros(us as u64);
            let start = Instant::now();
            loop {
                // Yield to other tasks
                self.yield_task();

                let elapsed = start.elapsed();
                if elapsed.as_micros() > us as u64 {
                    break;
                }

                let remaining = sleep_for - elapsed;

                if remaining < MIN_YIELD_DURATION {
                    // If the remaining time is less than the minimum yield time, we can just sleep
                    // for the remaining time.
                    unsafe { esp_rom_delay_us(remaining.as_micros() as u32) };
                    break;
                }
            }
        }
    }

    fn now(&self) -> u64 {
        // We're using a SingleShotTimer as the time driver, which lets us use the system timer's
        // timestamps.
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros()
    }
}

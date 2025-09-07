use core::{ffi::c_void, ptr::NonNull};

use allocator_api2::boxed::Box;
use esp_hal::time::{Duration, Instant};
use esp_radio_preempt_driver::semaphore::{SemaphoreImplementation, SemaphorePtr};
use esp_sync::NonReentrantMutex;

use crate::{
    InternalMemory,
    TICK_RATE,
    semaphore::Semaphore,
    task::{
        self,
        Context,
        TaskAllocListElement,
        TaskDeleteListElement,
        TaskList,
        TaskPtr,
        TaskQueue,
        TaskReadyListElement,
    },
    timer::TimeDriver,
    timer_queue,
};

pub(crate) struct SchedulerState {
    /// Pointer to the current task.
    pub(crate) current_task: Option<TaskPtr>,

    /// A list of all allocated tasks
    pub(crate) all_tasks: TaskList<TaskAllocListElement>,

    /// A list of tasks ready to run
    pub(crate) ready_tasks: TaskQueue<TaskReadyListElement>,

    /// Pointer to the task that is scheduled for deletion.
    pub(crate) to_delete: TaskList<TaskDeleteListElement>,

    pub(crate) time_driver: Option<TimeDriver>,
}

unsafe impl Send for SchedulerState {}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            current_task: None,
            all_tasks: TaskList::new(),
            ready_tasks: TaskQueue::new(),
            to_delete: TaskList::new(),

            time_driver: None,
        }
    }

    pub(crate) fn set_time_driver(&mut self, driver: TimeDriver) {
        self.time_driver = Some(driver);
    }

    fn delete_marked_tasks(&mut self) {
        while let Some(to_delete) = self.to_delete.pop() {
            self.all_tasks.remove(to_delete);
            self.ready_tasks.remove(to_delete);

            unsafe {
                let task = Box::from_raw_in(to_delete.as_ptr(), InternalMemory);
                core::mem::drop(task);
            }
        }
    }

    fn select_next_task(&mut self, currently_active_task: TaskPtr) -> Option<TaskPtr> {
        // At least one task must be ready to run. If there are none, we don't switch tasks.
        let next = self.ready_tasks.pop()?;

        // TODO: do not mark current task immediately ready to run
        self.ready_tasks.push(currently_active_task);

        if next == currently_active_task {
            return None;
        }

        Some(next)
    }

    #[cfg(xtensa)]
    pub(crate) fn switch_task(&mut self, trap_frame: &mut esp_hal::trapframe::TrapFrame) {
        self.delete_marked_tasks();
        let mut current_task = unwrap!(self.current_task);

        if let Some(mut next_task) = self.select_next_task(current_task) {
            task::save_task_context(unsafe { current_task.as_mut() }, trap_frame);
            task::restore_task_context(unsafe { next_task.as_mut() }, trap_frame);

            self.current_task = Some(next_task);
        }

        self.arm_time_slice_alarm();
    }

    #[cfg(riscv)]
    pub(crate) fn switch_task(&mut self) {
        self.delete_marked_tasks();
        let mut current = unwrap!(self.current_task);

        if let Some(mut next) = self.select_next_task(current) {
            let old_ctx = unsafe { &raw mut current.as_mut().trap_frame };
            let new_ctx = unsafe { &raw mut next.as_mut().trap_frame };

            if crate::task::arch_specific::task_switch(old_ctx, new_ctx) {
                self.current_task = Some(next);
            }
        }

        self.arm_time_slice_alarm();
    }

    fn arm_time_slice_alarm(&mut self) {
        let ready_tasks = !self.ready_tasks.is_empty();
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
        255
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
            state.ready_tasks.push(task_ptr);
        });

        task_ptr.as_ptr().cast()
    }

    fn current_task(&self) -> *mut c_void {
        task::current_task() as *mut c_void
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

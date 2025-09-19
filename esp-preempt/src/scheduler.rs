use core::{ffi::c_void, ptr::NonNull};

use allocator_api2::boxed::Box;
use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};
use esp_radio_preempt_driver::semaphore::{SemaphoreImplementation, SemaphoreKind, SemaphorePtr};
use esp_sync::NonReentrantMutex;

use crate::{
    InternalMemory,
    run_queue::{MaxPriority, RunQueue},
    semaphore::Semaphore,
    task::{
        self,
        CpuContext,
        Task,
        TaskAllocListElement,
        TaskDeleteListElement,
        TaskExt,
        TaskList,
        TaskPtr,
    },
    timer::TimeDriver,
};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// Due to how our interrupts work, we may have a timer event and a yield at the same time. Their
// order of processing is an implementation detail in esp-hal. We need to be able to store multiple
// events to not miss any.
pub(crate) struct SchedulerEvent(usize);

impl SchedulerEvent {
    // If this is NOT set, the event is a cooperative yield of some sort (time slicing, self-yield).
    const TASK_BLOCKED: usize = 1 << 0;

    fn contains(self, bit: usize) -> bool {
        self.0 & bit != 0
    }
    fn set(&mut self, bit: usize) {
        self.0 |= bit;
    }

    pub(crate) fn set_blocked(&mut self) {
        self.set(Self::TASK_BLOCKED)
    }

    pub(crate) fn is_cooperative_yield(self) -> bool {
        !self.contains(Self::TASK_BLOCKED)
    }
}

pub(crate) struct SchedulerState {
    /// The CPU on which the scheduler is running.
    pub(crate) runs_on: Cpu,

    /// Pointer to the current task.
    pub(crate) current_task: Option<TaskPtr>,
    idle_context: CpuContext,

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
            runs_on: Cpu::ProCpu,
            current_task: None,
            idle_context: CpuContext::new(),
            all_tasks: TaskList::new(),
            run_queue: RunQueue::new(),
            to_delete: TaskList::new(),

            time_driver: None,
            event: SchedulerEvent(0),
        }
    }

    pub(crate) fn setup(&mut self, time_driver: TimeDriver) {
        self.time_driver = Some(time_driver);
        self.runs_on = Cpu::current();
        task::set_idle_hook_entry(&mut self.idle_context);
    }

    pub(crate) fn create_task(
        &mut self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: usize,
    ) -> TaskPtr {
        let mut task = Box::new_in(
            Task::new(task, param, task_stack_size, priority),
            InternalMemory,
        );
        task.heap_allocated = true;
        let task_ptr = NonNull::from(Box::leak(task));

        self.all_tasks.push(task_ptr);
        if self.run_queue.mark_task_ready(task_ptr) {
            task::yield_task();
        }

        debug!("Task created: {:?}", task_ptr);

        task_ptr
    }

    fn delete_marked_tasks(&mut self) {
        while let Some(to_delete) = self.to_delete.pop() {
            trace!("delete_marked_tasks {:?}", to_delete);

            if Some(to_delete) == self.current_task {
                self.current_task = None;
            }
            self.delete_task(to_delete);
        }
    }

    fn run_scheduler(&mut self, task_switch: impl FnOnce(*mut CpuContext, *mut CpuContext)) {
        let mut priority = if let Some(current_task) = self.current_task {
            let current_task = unsafe { current_task.as_ref() };
            current_task.ensure_no_stack_overflow();
            Some(current_task.priority)
        } else {
            None
        };

        self.delete_marked_tasks();

        let event = core::mem::take(&mut self.event);

        if event.is_cooperative_yield()
            && let Some(current_task) = self.current_task
        {
            // Current task is still ready, mark it as such.
            debug!("re-queueing current task: {:?}", current_task);
            self.run_queue.mark_task_ready(current_task);
        }

        let next_task = self.run_queue.pop();
        if next_task != self.current_task {
            trace!("Switching task {:?} -> {:?}", self.current_task, next_task);

            // If the current task is deleted, we can skip saving its context. We signal this by
            // using a null pointer.
            let current_context = if let Some(mut current) = self.current_task {
                unsafe { &raw mut current.as_mut().cpu_context }
            } else {
                core::ptr::null_mut()
            };

            let next_context = if let Some(mut next) = next_task {
                priority = Some(next.priority(&mut self.run_queue));
                unsafe { &raw mut next.as_mut().cpu_context }
            } else if current_context.is_null() {
                // The current task has been deleted, we can't use its stack. Let's assume the main
                // task's context has a valid stack pointer for us.
                cfg_if::cfg_if! {
                    if #[cfg(xtensa)] {
                        self.idle_context.A1 = unsafe { task::MAIN_TASK.cpu_context.A1 };
                    } else {
                        self.idle_context.sp = unsafe { task::MAIN_TASK.cpu_context.sp };
                    }
                }
                &raw mut self.idle_context
            } else {
                // As the stack pointer, let's just use the current value.
                cfg_if::cfg_if! {
                    if #[cfg(xtensa)] {
                        self.idle_context.A1 = esp_hal::xtensa_lx::get_stack_pointer() as u32;
                    } else {
                        let current_sp;
                        unsafe { core::arch::asm!("mv {0}, sp", out(reg) current_sp); }
                        self.idle_context.sp = current_sp;
                    }
                }
                &raw mut self.idle_context
            };

            task_switch(current_context, next_context);

            // If we went to idle, this will be None and we won't mess up the main task's stack.
            self.current_task = next_task;
        }

        // TODO maybe we don't need to do this every time?
        // The current task is not in the run queue. If the run queue on the current priority level
        // is empty, the current task is the only one running at its priority level. In this
        // case, we don't need time slicing.
        let arm_next_timeslice_tick = priority
            .map(|p| !self.run_queue.is_level_empty(p))
            .unwrap_or(false);
        let time_driver = unwrap!(self.time_driver.as_mut());
        time_driver.arm_next_wakeup(arm_next_timeslice_tick);
    }

    pub(crate) fn switch_task(&mut self, #[cfg(xtensa)] trap_frame: &mut CpuContext) {
        self.run_scheduler(|current_context, next_context| {
            task::task_switch(
                current_context,
                next_context,
                #[cfg(xtensa)]
                trap_frame,
            )
        });
    }

    pub(crate) fn schedule_task_deletion(&mut self, task_to_delete: *mut Task) -> bool {
        let current_task = unwrap!(self.current_task);
        let task_to_delete = NonNull::new(task_to_delete).unwrap_or(current_task);
        let is_current = task_to_delete == current_task;

        self.to_delete.push(task_to_delete);

        is_current
    }

    pub(crate) fn sleep_until(&mut self, at: Instant) -> bool {
        let current_task = unwrap!(self.current_task);
        let timer_queue = unwrap!(self.time_driver.as_mut());
        if timer_queue.schedule_wakeup(current_task, at) {
            // The task has been scheduled for wakeup.
            self.event.set_blocked();
            task::yield_task();
            true
        } else {
            // The task refuses to sleep.
            false
        }
    }

    pub(crate) fn resume_task(&mut self, task: TaskPtr) {
        let timer_queue = unwrap!(self.time_driver.as_mut());
        timer_queue.timer_queue.remove(task);

        if self.run_queue.mark_task_ready(task) {
            task::yield_task();
        }
    }

    fn delete_task(&mut self, mut to_delete: TaskPtr) {
        self.remove_from_all_queues(to_delete);

        unsafe {
            if to_delete.as_ref().heap_allocated {
                let task = Box::from_raw_in(to_delete.as_ptr(), InternalMemory);
                core::mem::drop(task);
            } else {
                to_delete.as_mut().thread_semaphore = None;
            }
        }
    }

    fn remove_from_all_queues(&mut self, mut task: TaskPtr) {
        self.all_tasks.remove(task);
        unwrap!(self.time_driver.as_mut()).timer_queue.remove(task);

        if let Some(mut containing_queue) = unsafe { task.as_mut().current_queue.take() } {
            unsafe {
                containing_queue.as_mut().remove(task);
            }
        } else {
            self.run_queue.remove(task);
        }
    }
}

pub(crate) struct Scheduler {
    inner: NonReentrantMutex<SchedulerState>,
}

impl Scheduler {
    pub(crate) fn with<R>(&self, cb: impl FnOnce(&mut SchedulerState) -> R) -> R {
        self.inner.with(cb)
    }

    pub(crate) fn current_task(&self) -> TaskPtr {
        task::current_task()
    }

    pub(crate) fn create_task(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: u32,
    ) -> TaskPtr {
        self.with(|state| state.create_task(task, param, task_stack_size, priority as usize))
    }

    pub(crate) fn sleep_until(&self, wake_at: Instant) -> bool {
        self.with(|scheduler| scheduler.sleep_until(wake_at))
    }
}

esp_radio_preempt_driver::scheduler_impl!(pub(crate) static SCHEDULER: Scheduler = Scheduler {
    inner: NonReentrantMutex::new(SchedulerState::new())
});

impl esp_radio_preempt_driver::Scheduler for Scheduler {
    fn initialized(&self) -> bool {
        self.with(|scheduler| {
            if scheduler.time_driver.is_none() {
                warn!("Trying to initialize esp-radio before starting esp-preempt");
                return false;
            }

            if scheduler.runs_on != Cpu::current() {
                warn!(
                    "Trying to initialize esp-radio on {:?} but esp-preempt is running on {:?}",
                    Cpu::current(),
                    scheduler.runs_on
                );
                return false;
            }

            true
        })
    }

    fn yield_task(&self) {
        task::yield_task();
    }

    fn yield_task_from_isr(&self) {
        task::yield_task();
    }

    fn max_task_priority(&self) -> u32 {
        MaxPriority::MAX_PRIORITY as u32
    }

    fn task_create(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        priority: u32,
        _pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> *mut c_void {
        self.create_task(
            task,
            param,
            task_stack_size,
            priority.min(self.max_task_priority()),
        )
        .as_ptr()
        .cast()
    }

    fn current_task(&self) -> *mut c_void {
        self.current_task().as_ptr().cast()
    }

    fn schedule_task_deletion(&self, task_handle: *mut c_void) {
        task::schedule_task_deletion(task_handle as *mut Task)
    }

    fn current_task_thread_semaphore(&self) -> SemaphorePtr {
        task::with_current_task(|task| {
            if task.thread_semaphore.is_none() {
                task.thread_semaphore = Some(Semaphore::create(SemaphoreKind::Counting {
                    max: 1,
                    initial: 0,
                }));
            }

            unwrap!(task.thread_semaphore)
        })
    }

    fn usleep(&self, us: u32) {
        SCHEDULER.sleep_until(Instant::now() + Duration::from_micros(us as u64));
    }

    fn now(&self) -> u64 {
        // We're using a SingleShotTimer as the time driver, which lets us use the system timer's
        // timestamps.
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros()
    }
}

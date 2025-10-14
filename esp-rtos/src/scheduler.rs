use core::cell::RefCell;
#[cfg(feature = "esp-radio")]
use core::{ffi::c_void, ptr::NonNull};

#[cfg(feature = "alloc")]
use allocator_api2::boxed::Box;
use embassy_sync::blocking_mutex::Mutex;
use esp_hal::{system::Cpu, time::Instant};
use esp_sync::RawMutex;

#[cfg(feature = "alloc")]
use crate::InternalMemory;
#[cfg(feature = "rtos-trace")]
use crate::TraceEvents;
use crate::{
    run_queue::{Priority, RunQueue, RunSchedulerOn},
    task::{
        self,
        CpuContext,
        IdleFn,
        Task,
        TaskAllocListElement,
        TaskDeleteListElement,
        TaskExt,
        TaskList,
        TaskListItem,
        TaskPtr,
        TaskState,
    },
    timer::TimeDriver,
};

pub(crate) struct SchedulerState {
    /// A list of all allocated tasks
    pub(crate) all_tasks: TaskList<TaskAllocListElement>,

    /// A list of tasks ready to run
    pub(crate) run_queue: RunQueue,

    /// Pointer to the task that is scheduled for deletion.
    pub(crate) to_delete: TaskList<TaskDeleteListElement>,

    pub(crate) time_driver: Option<TimeDriver>,

    pub(crate) per_cpu: [CpuSchedulerState; Cpu::COUNT],
}

pub(crate) struct CpuSchedulerState {
    pub(crate) initialized: bool,
    /// Pointer to the current task.
    pub(crate) current_task: Option<TaskPtr>,
    idle_context: CpuContext,

    // This context will be filled out by the first context switch.
    // We allocate the main task statically, because there is always a main task. If deleted, we
    // simply don't deallocate this.
    pub(crate) main_task: Task,
}

impl CpuSchedulerState {
    const fn new() -> Self {
        Self {
            initialized: false,
            current_task: None,
            idle_context: CpuContext::new(),

            main_task: Task {
                cpu_context: CpuContext::new(),
                #[cfg(feature = "esp-radio")]
                thread_semaphore: None,
                state: TaskState::Ready,
                stack: core::ptr::slice_from_raw_parts_mut(core::ptr::null_mut(), 0),
                #[cfg(any(hw_task_overflow_detection, sw_task_overflow_detection))]
                stack_guard: core::ptr::null_mut(),
                #[cfg(sw_task_overflow_detection)]
                stack_guard_value: 0,
                current_queue: None,
                priority: Priority::ZERO,
                #[cfg(multi_core)]
                pinned_to: None,

                wakeup_at: 0,
                run_queued: false,
                timer_queued: false,

                alloc_list_item: TaskListItem::None,
                ready_queue_item: TaskListItem::None,
                timer_queue_item: TaskListItem::None,
                delete_list_item: TaskListItem::None,

                #[cfg(feature = "alloc")]
                heap_allocated: false,
            },
        }
    }

    #[cfg(multi_core)]
    pub fn current_priority(&self) -> Priority {
        self.current_task
            .map(|task| unsafe { (*task.as_ptr()).priority })
            .unwrap_or(Priority::ZERO)
    }
}

unsafe impl Send for SchedulerState {}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            all_tasks: TaskList::new(),
            run_queue: RunQueue::new(),
            to_delete: TaskList::new(),

            time_driver: None,

            per_cpu: [const { CpuSchedulerState::new() }; Cpu::COUNT],
        }
    }

    pub(crate) fn current_task(&self, cpu: Cpu) -> TaskPtr {
        unwrap!(
            self.per_cpu[cpu as usize].current_task,
            "The scheduler is not running on the current CPU. Make sure you start the scheduler before calling OS functions."
        )
    }

    pub(crate) fn setup(&mut self, time_driver: TimeDriver, idle_hook: IdleFn) {
        assert!(
            self.time_driver.is_none(),
            "The scheduler has already been started"
        );
        self.time_driver = Some(time_driver);
        for cpu in 0..Cpu::COUNT {
            task::set_idle_hook_entry(&mut self.per_cpu[cpu].idle_context, idle_hook);
        }
    }

    #[cfg(feature = "esp-radio")]
    pub(crate) fn create_task(
        &mut self,
        name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: usize,
        pinned_to: Option<Cpu>,
    ) -> TaskPtr {
        if let Some(cpu) = pinned_to {
            assert!(
                self.per_cpu[cpu as usize].initialized,
                "Cannot create task on uninitialized CPU"
            );
        }

        let mut task = Box::new_in(
            Task::new(name, task, param, task_stack_size, priority, pinned_to),
            InternalMemory,
        );
        task.heap_allocated = true;
        let task_ptr = NonNull::from(Box::leak(task));

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_new(task_ptr.rtos_trace_id());

        self.all_tasks.push(task_ptr);
        match self.run_queue.mark_task_ready(&self.per_cpu, task_ptr) {
            RunSchedulerOn::DontRun => {}
            RunSchedulerOn::CurrentCore => task::yield_task(),
            #[cfg(multi_core)]
            RunSchedulerOn::OtherCore => task::schedule_other_core(),
        }

        debug!("Task '{}' created: {:?}", name, task_ptr);

        task_ptr
    }

    #[cold]
    #[inline(never)]
    fn delete_marked_tasks(&mut self, cpu: Cpu) {
        let current_cpu = cpu as usize;
        let mut to_delete = core::mem::take(&mut self.to_delete);
        'outer: while let Some(task_ptr) = to_delete.pop() {
            assert!(task_ptr.state() == TaskState::Deleted);
            for cpu in 0..Cpu::COUNT {
                if cpu != current_cpu && Some(task_ptr) == self.per_cpu[cpu].current_task {
                    // We can't delete a task that is currently running on another CPU.
                    self.to_delete.push(task_ptr);
                    continue 'outer;
                }
            }

            trace!("delete_marked_tasks {:?}", task_ptr);
            self.delete_task(task_ptr);
        }
    }

    fn run_scheduler(&mut self, task_switch: impl FnOnce(*mut CpuContext, *mut CpuContext)) {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker_begin(TraceEvents::RunSchedule as u32);

        let cpu = Cpu::current();
        let current_cpu = cpu as usize;

        if !self.to_delete.is_empty() {
            self.delete_marked_tasks(cpu);
        }

        let current_task = self.per_cpu[current_cpu].current_task;
        if let Some(current_task) = current_task {
            unsafe { current_task.as_ref().ensure_no_stack_overflow() };

            if current_task.state() == TaskState::Ready {
                // Current task is still ready, mark it as such.
                debug!("re-queueing current task: {:?}", current_task);
                self.run_queue.mark_task_ready(&self.per_cpu, current_task);
            }
        };

        let mut arm_next_timeslice_tick = false;
        let next_task = self.run_queue.pop();
        if next_task != current_task {
            trace!("Switching task {:?} -> {:?}", current_task, next_task);

            // If the current task is deleted, we can skip saving its context. We signal this by
            // using a null pointer.
            let current_context = if let Some(current) = current_task {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_end(); // FIXME: rtos-trace should take the task ID for multi-core

                // TODO: the SMP scheduler relies on at least the context saving to happen within
                // the scheduler's critical section. We can't run the scheduler on the other core
                // while it might try to restore a partially saved context.
                #[cfg(multi_core)]
                let current_ref = unsafe { current.as_ref() };
                #[cfg(multi_core)]
                if current_ref.pinned_to.is_none()
                    && current_ref.priority >= self.per_cpu[1 - current_cpu].current_priority()
                {
                    task::schedule_other_core();
                }

                unsafe { &raw mut (*current.as_ptr()).cpu_context }
            } else {
                core::ptr::null_mut()
            };

            let next_context = if let Some(next) = next_task {
                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::task_exec_begin(next.rtos_trace_id());

                unsafe { next.as_ref().set_up_stack_watchpoint() };

                // If there are more tasks at this priority level, we need to schedule a timeslice
                // tick.
                let new_core_priority = next.priority(&mut self.run_queue);
                arm_next_timeslice_tick = !self.run_queue.is_level_empty(new_core_priority);

                unsafe { &raw mut (*next.as_ptr()).cpu_context }
            } else {
                // If there is no next task, set up and return to the idle hook.
                // Reuse the stack frame of the main task. Note that this requires the main task to
                // be pinned to the current CPU. If we're switching out the main task, however, we
                // can't rely on its saved context - use the current stack pointer which will still
                // point to the right stack, just to another place we can use within it.
                let idle_sp = if current_context.is_null()
                    || current_context == &raw mut self.per_cpu[current_cpu].main_task.cpu_context
                {
                    // We're using the current task's stack, for which the watchpoint is already set
                    // up.

                    let current_sp;
                    cfg_if::cfg_if! {
                        if #[cfg(xtensa)] {
                            unsafe { core::arch::asm!("mov {0}, sp", out(reg) current_sp); }
                        } else {
                            unsafe { core::arch::asm!("mv {0}, sp", out(reg) current_sp); }
                        }
                    }
                    current_sp
                } else {
                    // We're using the main task's stack.
                    self.per_cpu[current_cpu]
                        .main_task
                        .set_up_stack_watchpoint();

                    cfg_if::cfg_if! {
                        if #[cfg(xtensa)] {
                            self.per_cpu[current_cpu].main_task.cpu_context.A1
                        } else {
                            self.per_cpu[current_cpu].main_task.cpu_context.sp
                        }
                    }
                };

                cfg_if::cfg_if! {
                    if #[cfg(xtensa)] {
                        self.per_cpu[current_cpu].idle_context.A1 = idle_sp;
                    } else {
                        self.per_cpu[current_cpu].idle_context.sp = idle_sp;
                    }
                }

                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::system_idle();

                &raw mut self.per_cpu[current_cpu].idle_context
            };

            task_switch(current_context, next_context);

            // If we went to idle, this will be None and we won't mess up the main task's stack.
            self.per_cpu[current_cpu].current_task = next_task;
        }

        let time_driver = unwrap!(self.time_driver.as_mut());
        let now = crate::now();
        time_driver.set_time_slice(cpu, now, arm_next_timeslice_tick);
        time_driver.arm_next_wakeup(now);

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker_end(TraceEvents::RunSchedule as u32);
    }

    pub(crate) fn switch_task(&mut self, #[cfg(xtensa)] trap_frame: &mut CpuContext) {
        self.run_scheduler(|current_context, next_context| {
            trace!(
                "Task switch: {:x} -> {:x}",
                current_context as usize, next_context as usize
            );
            task::task_switch(
                current_context,
                next_context,
                #[cfg(xtensa)]
                trap_frame,
            )
        });
    }

    #[cfg(feature = "esp-radio")]
    pub(crate) fn schedule_task_deletion(&mut self, task_to_delete: *mut Task) -> bool {
        let current_cpu = Cpu::current() as usize;
        let current_task = unwrap!(self.per_cpu[current_cpu].current_task);
        let task_to_delete = NonNull::new(task_to_delete).unwrap_or(current_task);
        let is_current = task_to_delete == current_task;

        self.remove_from_all_queues(task_to_delete);
        if task_to_delete.state() != TaskState::Deleted {
            self.to_delete.push(task_to_delete);
            task_to_delete.set_state(TaskState::Deleted);
        }

        if is_current {
            self.per_cpu[current_cpu].current_task = None;
        }

        is_current
    }

    pub(crate) fn sleep_task_until(&mut self, task: TaskPtr, at: Instant) -> bool {
        let timer_queue = unwrap!(self.time_driver.as_mut());
        timer_queue.schedule_wakeup(task, at)
    }

    pub(crate) fn resume_task(&mut self, task: TaskPtr) {
        let timer_queue = unwrap!(self.time_driver.as_mut());
        timer_queue.timer_queue.remove(task);

        match self.run_queue.mark_task_ready(&self.per_cpu, task) {
            RunSchedulerOn::DontRun => {}
            RunSchedulerOn::CurrentCore => task::yield_task(),
            #[cfg(multi_core)]
            RunSchedulerOn::OtherCore => task::schedule_other_core(),
        }
    }

    fn delete_task(&mut self, mut to_delete: TaskPtr) {
        unsafe { to_delete.as_ref().ensure_no_stack_overflow() };

        unsafe {
            #[cfg(feature = "alloc")]
            if to_delete.as_ref().heap_allocated {
                let task = Box::from_raw_in(to_delete.as_ptr(), InternalMemory);
                core::mem::drop(task);
                return;
            }

            core::ptr::drop_in_place(to_delete.as_mut());
        }
    }

    #[cfg(feature = "esp-radio")]
    fn remove_from_all_queues(&mut self, mut task: TaskPtr) {
        self.all_tasks.remove(task);
        unwrap!(self.time_driver.as_mut()).timer_queue.remove(task);

        if let Some(mut containing_queue) = unsafe { task.as_mut().current_queue.take() } {
            unsafe { containing_queue.as_mut().remove(task) };
        } else {
            self.run_queue.remove(task);
        }
    }
}

pub(crate) struct Scheduler {
    inner: Mutex<RawMutex, RefCell<SchedulerState>>,
}

impl Scheduler {
    pub(crate) fn with<R>(&self, cb: impl FnOnce(&mut SchedulerState) -> R) -> R {
        self.with_shared(|shared| cb(&mut *unwrap!(shared.try_borrow_mut())))
    }

    pub(crate) fn with_shared<R>(&self, cb: impl FnOnce(&RefCell<SchedulerState>) -> R) -> R {
        self.inner.lock(|shared| cb(shared))
    }

    #[cfg(feature = "esp-radio")]
    pub(crate) fn current_task(&self) -> TaskPtr {
        task::current_task()
    }

    #[cfg(feature = "esp-radio")]
    pub(crate) fn create_task(
        &self,
        name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: u32,
        pinned_to: Option<Cpu>,
    ) -> TaskPtr {
        self.with(|state| {
            state.create_task(
                name,
                task,
                param,
                task_stack_size,
                priority as usize,
                pinned_to,
            )
        })
    }

    pub(crate) fn sleep_until(&self, wake_at: Instant) -> bool {
        self.with(|scheduler| {
            let current_task = scheduler.current_task(Cpu::current());
            if scheduler.sleep_task_until(current_task, wake_at) {
                task::yield_task();
                true
            } else {
                false
            }
        })
    }
}

#[cfg(feature = "esp-radio")]
esp_radio_rtos_driver::scheduler_impl!(pub(crate) static SCHEDULER: Scheduler = Scheduler {
    inner: Mutex::new(RefCell::new(SchedulerState::new()))
});

#[cfg(not(feature = "esp-radio"))]
pub(crate) static SCHEDULER: Scheduler = Scheduler {
    inner: Mutex::new(RefCell::new(SchedulerState::new())),
};

#[cfg(feature = "rtos-trace")]
impl rtos_trace::RtosTraceOSCallbacks for Scheduler {
    fn task_list() {
        SCHEDULER.with(|s| {
            for task in s.all_tasks.iter() {
                rtos_trace::trace::task_send_info(
                    task.rtos_trace_id(),
                    task.rtos_trace_info(&mut s.run_queue),
                );
            }
        })
    }

    fn time() -> u64 {
        crate::now()
    }
}

#[cfg(feature = "rtos-trace")]
rtos_trace::global_os_callbacks!(Scheduler);

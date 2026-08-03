#[cfg(feature = "esp-radio")]
use core::ffi::c_void;
use core::{
    cell::{RefCell, RefMut},
    ptr::NonNull,
};

#[cfg(feature = "alloc")]
use allocator_api2::boxed::Box;
use embassy_sync::blocking_mutex::Mutex;
use esp_hal::{system::Cpu, time::Instant};
use esp_sync::RawMutex;
use macros::ram;

#[cfg(feature = "alloc")]
use crate::InternalMemory;
#[cfg(feature = "rtos-trace")]
use crate::TraceEvents;
#[cfg(feature = "embassy")]
use crate::timer::embassy::TimerQueue;
use crate::{
    run_queue::{Priority, RunQueue},
    task::{
        self,
        ContextExt,
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
        ThreadLocalData,
        read_thread_pointer,
    },
    timer::TimeDriver,
};

pub(crate) struct SchedulerState {
    /// A list of all allocated tasks
    pub(crate) all_tasks: TaskList<TaskAllocListElement>,

    /// A list of tasks ready to run
    pub(crate) run_queue: RunQueue,

    pub(crate) time_driver: Option<TimeDriver>,

    pub(crate) per_cpu: [CpuState; Cpu::COUNT],
}

pub(crate) struct CpuState {
    pub(crate) initialized: bool,
    idle_context: CpuContext,

    /// A pointer to the current task.
    ///
    /// While the task pointer is available through the thread pointer register,
    /// sometimes we need to check the other core's task pointer, so we need a copy in memory.
    #[cfg(multi_core)]
    current_task: *mut Task,

    /// Pointer to the task that is scheduled for deletion.
    pub(crate) to_delete: TaskList<TaskDeleteListElement>,

    /// Set while the CPU executes the idle context.
    ///
    /// The idle context has no `Task`, so the thread pointer is null while it runs. A task that
    /// has deleted itself also has a null thread pointer, so the flag is needed to tell the two
    /// apart.
    idle: bool,

    // This context will be filled out by the first context switch.
    // We allocate the main task statically, because there is always a main task.
    // Cannot be deleted.
    pub(crate) main_task: Task,
}

impl CpuState {
    const fn new() -> Self {
        Self {
            initialized: false,
            idle_context: CpuContext::new(),
            to_delete: TaskList::new(),
            idle: false,

            #[cfg(multi_core)]
            current_task: core::ptr::null_mut(),

            main_task: Task {
                cpu_context: CpuContext::new(),
                thread_local: ThreadLocalData::new(),
                state: TaskState::Ready,
                stack: core::ptr::slice_from_raw_parts_mut(core::ptr::null_mut(), 0),
                #[cfg(any(hw_task_overflow_detection, sw_task_overflow_detection))]
                stack_guard: core::ptr::null_mut(),
                #[cfg(sw_task_overflow_detection)]
                stack_guard_value: 0,
                #[cfg(feature = "esp-radio")]
                current_wait_queue: None,
                priority: Priority::ZERO,
                #[cfg(multi_core)]
                pinned_to: None,

                wakeup_at: 0,
                in_run_or_wait_queue: false,
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
}

unsafe impl Send for SchedulerState {}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            all_tasks: TaskList::new(),
            run_queue: RunQueue::new(),

            time_driver: None,

            per_cpu: [const { CpuState::new() }; Cpu::COUNT],
        }
    }

    #[cfg(multi_core)]
    pub(crate) fn priority_of_core(per_cpu: &[CpuState], core: usize) -> Priority {
        unsafe { per_cpu[core].current_task.as_ref() }
            .map(|task| task.priority)
            .unwrap_or(Priority::ZERO)
    }

    #[inline]
    #[cfg(multi_core)]
    pub(crate) fn set_current_task(&mut self, cpu: Cpu, task: Option<TaskPtr>) {
        self.per_cpu[cpu as usize].current_task =
            task.map(|task| task.as_ptr()).unwrap_or_default();
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
        let mut task_ptr = NonNull::from(Box::leak(task));

        unsafe {
            task_ptr
                .as_mut()
                .cpu_context
                .set_tp(task_ptr.as_ptr() as u32)
        };

        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::task_new(task_ptr.rtos_trace_id());

        self.all_tasks.push(task_ptr);
        let run_scheduler = self.run_queue.mark_task_ready(&self.per_cpu, task_ptr);
        task::trigger_scheduler(run_scheduler);

        debug!("Task '{}' created: {:?}", name, task_ptr);

        task_ptr
    }

    /// Deletes the tasks marked for deletion on `cpu`, except the one that owns `current_sp`.
    ///
    /// A task that deletes itself keeps running on its own stack until the scheduler switches away
    /// from it. Freeing that stack here would hand it back to the allocator while this CPU still
    /// writes to it, and the other core could hand it out again. Such a task stays in the list, and
    /// a later scheduler run deletes it, once this CPU runs on a different stack.
    ///
    /// Only the task the CPU currently runs on can be deferred, so the list holds at most one task
    /// after this function returns.
    #[cold]
    #[inline(never)]
    fn delete_marked_tasks(&mut self, cpu: Cpu, current_sp: usize) {
        let mut in_use = None;

        while let Some(task_ptr) = self.per_cpu[cpu as usize].to_delete.pop() {
            assert!(task_ptr.state() == TaskState::Deleted);

            if unsafe { task_ptr.as_ref() }.owns_stack_pointer(current_sp) {
                trace!("delete_marked_tasks {:?} is still in use", task_ptr);
                in_use = Some(task_ptr);
                continue;
            }

            trace!("delete_marked_tasks {:?}", task_ptr);
            self.delete_task(task_ptr);
        }

        if let Some(task_ptr) = in_use {
            self.per_cpu[cpu as usize].to_delete.push(task_ptr);
        }
    }

    fn run_scheduler(&mut self, task_switch: impl FnOnce(*mut CpuContext, *mut CpuContext)) {
        #[cfg(feature = "rtos-trace")]
        rtos_trace::trace::marker_begin(TraceEvents::RunSchedule as u32);

        let cpu = Cpu::current();
        let current_cpu = cpu as usize;

        let current_sp: u32;
        cfg_select! {
            xtensa => unsafe {
                core::arch::asm!("mov {0}, sp", out(reg) current_sp);
            },
            _ => unsafe {
                core::arch::asm!("mv {0}, sp", out(reg) current_sp);
            },
        }

        if !self.per_cpu[cpu as usize].to_delete.is_empty() {
            self.delete_marked_tasks(cpu, current_sp as usize);
        }

        let current_task = NonNull::new(read_thread_pointer());

        // A task that deleted itself has no thread pointer, but it keeps running on its own stack.
        // We have to switch away from it, even if there is nothing else to run, so that we do not
        // return to a task that no longer exists.
        let deleted_self = current_task.is_none() && !self.per_cpu[current_cpu].idle;

        // The idle task has no Task structure, and it has no stack of its own - it runs on the
        // main task's stack. Check the main task in that case, so that a deep idle hook cannot
        // overflow the main stack unnoticed. Before the main task is set up, there is no stack
        // guard to check. A task that deleted itself also has no thread pointer, but it still runs
        // on its own stack, which is about to be freed - there is nothing to check for it.
        let stack_owner = match current_task {
            Some(current_task) => Some(current_task),
            None if self.per_cpu[current_cpu].idle => {
                Some(NonNull::from(&self.per_cpu[current_cpu].main_task))
            }
            None => None,
        };
        if let Some(stack_owner) = stack_owner {
            unsafe {
                stack_owner
                    .as_ref()
                    .ensure_no_stack_overflow(current_sp as usize)
            };
        }

        if let Some(current_task) = current_task
            && current_task.state() == TaskState::Ready
        {
            // Current task is still ready, mark it as such.
            debug!("re-queueing current task: {:?}", current_task);
            self.run_queue.mark_task_ready(&self.per_cpu, current_task);
        }

        let mut arm_next_timeslice_tick = false;
        let next_task = self.run_queue.pop();
        if next_task != current_task || deleted_self {
            debug!("Switching task {:?} -> {:?}", current_task, next_task);

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
                    && current_ref.priority
                        >= Self::priority_of_core(&self.per_cpu, 1 - current_cpu)
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
                let idle_sp = if current_context
                    == &raw mut self.per_cpu[current_cpu].main_task.cpu_context
                {
                    // We're using the current task's stack, for which the watchpoint is already set
                    // up.
                    current_sp
                } else {
                    // We're using the main task's stack.
                    self.per_cpu[current_cpu]
                        .main_task
                        .set_up_stack_watchpoint();

                    self.per_cpu[current_cpu].main_task.cpu_context.sp()
                };

                self.per_cpu[current_cpu].idle_context.set_sp(idle_sp);

                #[cfg(feature = "rtos-trace")]
                rtos_trace::trace::system_idle();

                &raw mut self.per_cpu[current_cpu].idle_context
            };

            self.per_cpu[current_cpu].idle = next_task.is_none();

            task_switch(current_context, next_context);

            // If we went to idle, this will be None and we won't mess up the main task's stack.
            #[cfg(multi_core)]
            self.set_current_task(cpu, next_task);
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

    /// Returns whether `task` is the main task of any CPU.
    #[cfg(feature = "esp-radio")]
    fn is_main_task(&self, task: TaskPtr) -> bool {
        self.per_cpu
            .iter()
            .any(|per_cpu| core::ptr::eq(task.as_ptr(), &raw const per_cpu.main_task))
    }

    /// Returns the CPU that runs `task`, if that CPU is not the current one.
    #[cfg(all(multi_core, feature = "esp-radio"))]
    fn other_cpu_running(&self, task: TaskPtr) -> Option<Cpu> {
        let current_cpu = Cpu::current();
        Cpu::all().find(|cpu| {
            *cpu != current_cpu
                && core::ptr::eq(self.per_cpu[*cpu as usize].current_task, task.as_ptr())
        })
    }

    #[cfg(feature = "esp-radio")]
    pub(crate) fn schedule_task_deletion(&mut self, task_to_delete: Option<TaskPtr>) -> bool {
        let current_task = SCHEDULER.current_task();
        let task_to_delete = task_to_delete.unwrap_or(current_task);

        // Every CPU must keep its main task. The idle context has no stack of its own and runs on
        // the main task's stack, so a CPU without a main task has nothing to fall back to.
        assert!(
            !self.is_main_task(task_to_delete),
            "The main task must not be deleted: {:?}",
            task_to_delete
        );

        let is_current = task_to_delete == current_task;

        self.remove_from_all_queues(task_to_delete);

        if is_current {
            self.mark_for_deletion(Cpu::current(), task_to_delete);

            crate::task::write_thread_pointer(core::ptr::null_mut());
        } else {
            cfg_select! {
                multi_core => {
                    // Another CPU may run this task. We must not free it, because that CPU still
                    // uses its stack, and because the allocator could hand the same address to a
                    // new task while that CPU still holds the address as its current task. Let the
                    // other CPU switch away from the task first, and delete the task there.
                    if let Some(cpu) = self.other_cpu_running(task_to_delete) {
                        self.mark_for_deletion(cpu, task_to_delete);
                        task::schedule_other_core();
                    } else {
                        self.delete_task(task_to_delete);
                    }
                }
                _ => {
                    // On a single CPU, a task that is not the current task does not run, so its
                    // stack is not in use.
                    self.delete_task(task_to_delete);
                }
            }
        }

        is_current
    }

    /// Marks `task` deleted, and queues it to be deleted by a scheduler run on `cpu`.
    #[cfg(feature = "esp-radio")]
    fn mark_for_deletion(&mut self, cpu: Cpu, task: TaskPtr) {
        if task.state() != TaskState::Deleted {
            self.per_cpu[cpu as usize].to_delete.push(task);
            task.set_state(TaskState::Deleted);
        }
    }

    pub(crate) fn sleep_task_until(&mut self, task: TaskPtr, at: Instant) -> bool {
        let timer_queue = unwrap!(self.time_driver.as_mut());
        timer_queue.schedule_wakeup(task, at)
    }

    #[ram]
    pub(crate) fn resume_task(&mut self, task: TaskPtr) {
        let timer_queue = unwrap!(self.time_driver.as_mut());
        timer_queue.timer_queue.remove(task);

        let run_scheduler = self.run_queue.mark_task_ready(&self.per_cpu, task);
        task::trigger_scheduler(run_scheduler);
    }

    fn delete_task(&mut self, mut to_delete: TaskPtr) {
        unsafe {
            cfg_select! {
                xtensa => {
                    let saved_sp = to_delete.as_ref().cpu_context.A1 as usize;
                }
                _ => {
                    let saved_sp = to_delete.as_ref().cpu_context.sp;
                }
            }
            to_delete.as_ref().ensure_no_stack_overflow(saved_sp)
        };

        debug!("Dropping task: {:x}", to_delete.as_ptr() as usize);
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

        if let Some(mut containing_queue) = unsafe { task.as_mut().current_wait_queue.take() } {
            unsafe { containing_queue.as_mut().remove(task) };
        } else {
            self.run_queue.remove(task);
        }
    }

    pub(crate) fn set_priority(&mut self, mut task: TaskPtr, new_priority: Priority) {
        // If the task is in a run queue, it needs to be moved to the new priority's run queue.
        let task_in_run_queue = {
            let task = unsafe { task.as_ref() };
            let in_queue = task.in_run_or_wait_queue;

            let in_waitqueue = cfg_select! {
                feature = "esp-radio" => task.current_wait_queue.is_some(),
                _ => false,
            };

            in_queue && !in_waitqueue
        };

        if task_in_run_queue {
            self.run_queue.remove(task);
        }

        // Update priority.
        {
            let task = unsafe { task.as_mut() };
            task.priority = new_priority;
        }

        if task_in_run_queue {
            self.resume_task(task);
        }
    }

    #[cfg(all(multi_core, sleep_light_sleep))]
    pub(crate) fn cpu_idle(&self, cpu: Cpu) -> bool {
        let per_cpu = &self.per_cpu[cpu as usize];
        // A CPU that never started the scheduler has no work to do, so it counts as idle.
        !per_cpu.initialized || per_cpu.idle
    }
}

pub(crate) struct GlobalState {
    pub scheduler: RefCell<SchedulerState>,
    #[cfg(feature = "embassy")]
    pub embassy_timer_queue: RefCell<TimerQueue>,
}

impl GlobalState {
    const fn new() -> Self {
        Self {
            scheduler: RefCell::new(SchedulerState::new()),
            #[cfg(feature = "embassy")]
            embassy_timer_queue: RefCell::new(TimerQueue::new()),
        }
    }

    pub fn scheduler(&self) -> RefMut<'_, SchedulerState> {
        unwrap!(self.scheduler.try_borrow_mut())
    }

    #[cfg(feature = "embassy")]
    pub fn embassy_timer_queue(&self) -> RefMut<'_, TimerQueue> {
        unwrap!(self.embassy_timer_queue.try_borrow_mut())
    }
}

pub(crate) struct Scheduler {
    inner: Mutex<RawMutex, GlobalState>,
}

impl Scheduler {
    const fn new() -> Self {
        Self {
            inner: Mutex::new(GlobalState::new()),
        }
    }

    pub(crate) fn with<R>(&self, cb: impl FnOnce(&mut SchedulerState) -> R) -> R {
        self.with_shared(|shared| cb(&mut shared.scheduler()))
    }

    pub(crate) fn with_shared<R>(&self, cb: impl FnOnce(&GlobalState) -> R) -> R {
        self.inner.lock(cb)
    }

    pub(crate) fn current_task(&self) -> TaskPtr {
        let tp = read_thread_pointer();

        unwrap!(
            TaskPtr::new(tp),
            "The scheduler has not been started. Make sure to call `esp_rtos::init()` before trying to access the current task."
        )
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
            let current_task = SCHEDULER.current_task();
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
esp_radio_rtos_driver::register_scheduler_implementation!(pub(crate) static SCHEDULER: Scheduler = Scheduler::new());

#[cfg(not(feature = "esp-radio"))]
pub(crate) static SCHEDULER: Scheduler = Scheduler::new();

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

#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
pub(crate) mod arch_specific;

use core::{ffi::c_void, marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

#[cfg(feature = "alloc")]
use allocator_api2::boxed::Box;
pub(crate) use arch_specific::*;
use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};
#[cfg(feature = "esp-radio")]
use esp_radio_rtos_driver::semaphore::{SemaphoreHandle, SemaphorePtr};
#[cfg(feature = "rtos-trace")]
use rtos_trace::TaskInfo;

use crate::{
    SCHEDULER,
    run_queue::{Priority, RunQueue, RunSchedulerOn},
    scheduler::SchedulerState,
    thread::{ThreadOptions, ThreadPtr},
    wait_queue::WaitQueue,
};

pub type IdleFn = extern "C" fn() -> !;

pub(crate) extern "C" fn idle_hook() -> ! {
    loop {
        esp_hal::interrupt::wait_for_interrupt();
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum TaskState {
    Ready,
    Sleeping,
    Deleted,
}

pub(crate) type TaskPtr = NonNull<Task>;
pub(crate) type TaskListItem = Option<TaskPtr>;

/// An abstraction that allows the task to contain multiple different queue pointers.
pub(crate) trait TaskListElement: Default {
    /// Returns the pointer to the next element in the list.
    fn next(task: TaskPtr) -> Option<TaskPtr>;

    /// Sets the pointer to the next element in the list.
    fn set_next(task: TaskPtr, next: Option<TaskPtr>);

    /// Returns whether the task is in the list. If this function returns `None`, we don't know.
    fn is_in_queue(_task: TaskPtr) -> Option<bool> {
        // By default we don't store this information, so we return "Don't know".
        None
    }

    /// Marks whether the task is in the list.
    fn mark_in_queue(_task: TaskPtr, _in_queue: bool) {}
}

macro_rules! task_list_item {
    ($struct:ident, $field:ident $(, $in_queue_field:ident)?) => {
        #[derive(Default)]
        pub(crate) struct $struct;
        impl TaskListElement for $struct {
            fn next(task: TaskPtr) -> Option<TaskPtr> {
                unsafe { task.as_ref().$field }
            }

            fn set_next(mut task: TaskPtr, next: Option<TaskPtr>) {
                unsafe {
                    task.as_mut().$field = next;
                }
            }

            $(
                fn is_in_queue(task: TaskPtr) -> Option<bool> {
                    Some(unsafe { task.as_ref().$in_queue_field })
                }

                fn mark_in_queue(mut task: TaskPtr, in_queue: bool) {
                    unsafe {
                        task.as_mut().$in_queue_field = in_queue;
                    }
                }
            )?
        }
    };
}

task_list_item!(
    TaskReadyQueueElement,
    ready_queue_item,
    in_run_or_wait_queue
);
task_list_item!(TaskTimerQueueElement, timer_queue_item, timer_queued);
// These aren't perf critical, no need to waste memory on caching list status:
task_list_item!(TaskAllocListElement, alloc_list_item);
task_list_item!(TaskDeleteListElement, delete_list_item);

/// Extension trait for common task operations. These should be inherent methods but we can't
/// implement stuff for NonNull.
pub(crate) trait TaskExt {
    #[cfg(feature = "rtos-trace")]
    fn rtos_trace_id(self) -> u32;
    #[cfg(feature = "rtos-trace")]
    fn rtos_trace_info(self, run_queue: &mut RunQueue) -> TaskInfo;

    fn priority(self, _: &mut RunQueue) -> Priority;
    fn state(self) -> TaskState;
    fn set_state(self, state: TaskState);
}

impl TaskExt for TaskPtr {
    #[cfg(feature = "rtos-trace")]
    fn rtos_trace_id(self) -> u32 {
        self.addr().get() as u32
    }

    #[cfg(feature = "rtos-trace")]
    fn rtos_trace_info(self, run_queue: &mut RunQueue) -> TaskInfo {
        TaskInfo {
            name: unsafe { self.as_ref().name.unwrap_or("<unnamed>") },
            priority: self.priority(run_queue).get() as u32,
            stack_base: unsafe { self.as_ref().stack.addr() },
            stack_size: unsafe { self.as_ref().stack.len() },
        }
    }

    fn priority(self, _: &mut RunQueue) -> Priority {
        unsafe { self.as_ref().priority }
    }

    fn state(self) -> TaskState {
        unsafe { self.as_ref().state }
    }

    fn set_state(mut self, state: TaskState) {
        trace!("Task {:?} state changed to {:?}", self, state);

        #[cfg(feature = "rtos-trace")]
        match state {
            TaskState::Ready => rtos_trace::trace::task_ready_begin(self.rtos_trace_id()),
            TaskState::Sleeping => rtos_trace::trace::task_ready_end(self.rtos_trace_id()),
            TaskState::Deleted => rtos_trace::trace::task_terminate(self.rtos_trace_id()),
        }

        unsafe { self.as_mut().state = state };
    }
}

/// A singly linked list of tasks.
///
/// Use this where you don't care about the order of list elements.
///
/// The `E` type parameter is used to access the data in the task object that belongs to this list.
#[derive(Default)]
pub(crate) struct TaskList<E> {
    head: Option<TaskPtr>,
    _item: PhantomData<E>,
}

impl<E: TaskListElement> TaskList<E> {
    pub const fn new() -> Self {
        Self {
            head: None,
            _item: PhantomData,
        }
    }

    pub fn push(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(true) {
            return;
        }
        E::mark_in_queue(task, true);

        debug_assert!(E::next(task).is_none());
        E::set_next(task, self.head);
        self.head = Some(task);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        let popped = self.head.take();

        if let Some(task) = popped {
            self.head = E::next(task);
            E::set_next(task, None);
            E::mark_in_queue(task, false);
        }

        popped
    }

    pub fn remove(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(false) {
            return;
        }

        // TODO: maybe this (and TaskQueue::remove) may prove too expensive.
        let mut list = core::mem::take(self);
        while let Some(popped) = list.pop() {
            if popped != task {
                self.push(popped);
            } else {
                E::mark_in_queue(task, false);
            }
        }
    }

    #[cfg(feature = "rtos-trace")]
    pub fn iter(&self) -> impl Iterator<Item = TaskPtr> {
        let mut current = self.head;
        core::iter::from_fn(move || {
            let task = current?;
            current = E::next(task);
            Some(task)
        })
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}

/// A singly linked queue of tasks.
///
/// Use this where you care about the order of list elements. Elements are popped from the front,
/// and pushed to the back.
///
/// The `E` type parameter is used to access the data in the task object that belongs to this list.
#[derive(Default)]
pub(crate) struct TaskQueue<E> {
    head: Option<TaskPtr>,
    tail: Option<TaskPtr>,
    _item: PhantomData<E>,
}

impl<E: TaskListElement> TaskQueue<E> {
    pub const fn new() -> Self {
        Self {
            head: None,
            tail: None,
            _item: PhantomData,
        }
    }

    pub fn push(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(true) {
            return;
        }
        E::mark_in_queue(task, true);

        debug_assert!(E::next(task).is_none());
        if let Some(tail) = self.tail {
            E::set_next(tail, Some(task));
        } else {
            self.head = Some(task);
        }
        self.tail = Some(task);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        let popped = self.head.take();

        if let Some(task) = popped {
            self.head = E::next(task);
            E::set_next(task, None);
            if self.head.is_none() {
                self.tail = None;
            }
            E::mark_in_queue(task, false);
        }

        popped
    }

    pub fn pop_if(&mut self, cond: impl Fn(&Task) -> bool) -> Option<TaskPtr> {
        let mut popped = None;

        let mut list = core::mem::take(self);
        while let Some(task) = list.pop() {
            if popped.is_none() && cond(unsafe { task.as_ref() }) {
                E::mark_in_queue(task, false);
                popped = Some(task);
            } else {
                self.push(task);
            }
        }

        popped
    }

    pub fn remove(&mut self, task: TaskPtr) {
        if E::is_in_queue(task) == Some(false) {
            return;
        }

        _ = self.pop_if(|t| NonNull::from(t) == task);
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}

pub(crate) struct ThreadLocalData {
    #[cfg(feature = "esp-radio")]
    pub thread_semaphore: Option<SemaphorePtr>,

    // The _reent struct is rarely needed, but big. Let's heap-allocate it, to save a bit of RAM.
    #[cfg(feature = "alloc")]
    pub reent: Option<Box<esp_rom_sys::_reent>>,
}
impl ThreadLocalData {
    pub const fn new() -> Self {
        Self {
            #[cfg(feature = "esp-radio")]
            thread_semaphore: None,

            #[cfg(feature = "alloc")]
            reent: None,
        }
    }
}

impl Drop for ThreadLocalData {
    fn drop(&mut self) {
        #[cfg(feature = "esp-radio")]
        if let Some(semaphore_ptr) = self.thread_semaphore.take() {
            core::mem::drop(unsafe { SemaphoreHandle::from_ptr(semaphore_ptr) });
        }
    }
}

#[repr(C)]
pub(crate) struct Task {
    pub cpu_context: CpuContext,

    pub thread_local: ThreadLocalData,

    pub name: Option<&'static str>,

    pub state: TaskState,
    pub stack: *mut [MaybeUninit<u32>],

    #[cfg(any(hw_task_overflow_detection, sw_task_overflow_detection))]
    pub stack_guard: *mut u32,
    #[cfg(sw_task_overflow_detection)]
    pub(crate) stack_guard_value: u32,

    pub priority: Priority,
    #[cfg(multi_core)]
    pub pinned_to: Option<Cpu>,

    pub wakeup_at: u64,

    /// Whether the task is currently queued in the run queue.
    pub in_run_or_wait_queue: bool,
    /// Whether the task is currently queued in the timer queue.
    pub timer_queued: bool,

    /// The current wait queue this task is in.
    pub(crate) current_wait_queue: Option<NonNull<WaitQueue>>,

    // Lists a task can be in:
    /// The list of all allocated tasks
    pub alloc_list_item: TaskListItem,

    /// Either the RunQueue or the WaitQueue
    pub ready_queue_item: TaskListItem,

    /// The timer queue
    pub timer_queue_item: TaskListItem,

    /// The list of tasks scheduled for deletion
    pub delete_list_item: TaskListItem,

    /// The memory block this task lives in, for tasks that were not statically allocated by the
    /// scheduler. The main tasks have no such block.
    pub(crate) thread: Option<ThreadPtr>,
}

pub(crate) trait ContextExt {
    fn set_tp(&mut self, tp: u32);

    fn sp(&self) -> u32;

    fn set_sp(&mut self, sp: u32);
}

impl ContextExt for CpuContext {
    fn set_tp(&mut self, tp: u32) {
        cfg_select! {
            xtensa => {
                self.THREADPTR = tp;
            }
            riscv => {
                self.tp = tp as usize;
            }
            _ => {}
        }
    }

    fn sp(&self) -> u32 {
        cfg_select! {
            xtensa => self.A1,
            _ => self.sp as u32,
        }
    }

    fn set_sp(&mut self, sp: u32) {
        cfg_select! {
            xtensa => {
                self.A1 = sp;
            }
            _ => {
                self.sp = sp as usize;
            }
        }
    }
}

extern "C" fn task_wrapper(task_fn: extern "C" fn(*mut c_void), param: *mut c_void) {
    task_fn(param);
    schedule_task_deletion(None);
}

impl Task {
    /// Creates a task that runs `entry(param)` on a caller-provided stack.
    ///
    /// The stack guard is placed at the low end of `stack`, so the region must be larger than the
    /// stack guard offset. `thread` is the memory block the task lives in.
    pub(crate) fn new(
        options: &ThreadOptions,
        entry: extern "C" fn(*mut c_void),
        param: *mut c_void,
        stack: *mut [MaybeUninit<u32>],
        thread: ThreadPtr,
    ) -> Self {
        debug!(
            "task_create {:?} {:?}({:?}) stack = {:?} priority = {} pinned_to = {:?}",
            options.name, entry, param, stack, options.priority, options.pinned_to
        );

        let stack_guard_offset =
            esp_config::esp_config_int!(usize, "ESP_HAL_CONFIG_STACK_GUARD_OFFSET");

        let stack_bottom = stack.cast::<MaybeUninit<u32>>();
        let stack_top = unsafe { stack_bottom.add(stack.len()).cast() };

        let mut task = Task {
            cpu_context: new_task_context(entry, param, stack_top),
            thread_local: ThreadLocalData::new(),
            name: options.name,
            state: TaskState::Ready,
            stack,
            #[cfg(any(hw_task_overflow_detection, sw_task_overflow_detection))]
            stack_guard: stack.cast(),
            #[cfg(sw_task_overflow_detection)]
            stack_guard_value: 0,
            current_wait_queue: None,
            priority: Priority::new(options.priority),
            #[cfg(multi_core)]
            pinned_to: options.pinned_to,

            wakeup_at: 0,
            timer_queued: false,
            in_run_or_wait_queue: false,

            alloc_list_item: TaskListItem::None,
            ready_queue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,

            thread: Some(thread),
        };

        task.set_up_stack_guard(stack_guard_offset, 0xDEED_BAAD);

        task
    }

    fn set_up_stack_guard(&mut self, offset: usize, _value: u32) {
        let stack_bottom = self.stack.cast::<MaybeUninit<u32>>();
        let stack_guard = unsafe { stack_bottom.byte_add(offset) };

        #[cfg(sw_task_overflow_detection)]
        unsafe {
            // avoid touching the main stack's canary on the first core
            if stack_guard.read().assume_init() != _value {
                stack_guard.write(MaybeUninit::new(_value));
            }
            self.stack_guard_value = _value;
        }

        #[cfg(any(hw_task_overflow_detection, sw_task_overflow_detection))]
        {
            self.stack_guard = stack_guard.cast();
        }
    }

    /// Returns the address range of the task's stack, as `(bottom, top)`.
    ///
    /// The stack grows down from `top`, so `bottom` itself is not a valid stack pointer.
    fn stack_range(&self) -> (usize, usize) {
        let bottom = self.stack.cast::<MaybeUninit<u32>>();
        let top = bottom.wrapping_add(self.stack.len());
        (bottom as usize, top as usize)
    }

    /// Returns whether `sp` points into this task's stack.
    pub(crate) fn owns_stack_pointer(&self, sp: usize) -> bool {
        let (bottom, top) = self.stack_range();
        sp > bottom && sp <= top
    }

    pub(crate) fn ensure_no_stack_overflow(&self, _sp: usize) {
        #[cfg(sw_task_overflow_detection)]
        assert_eq!(
            // This cast is safe to do from MaybeUninit<u32> because this is the word we've written
            // during initialization.
            unsafe { self.stack_guard.read() },
            self.stack_guard_value,
            "Stack overflow detected in {:?}",
            self as *const Task
        );

        #[cfg(stack_pointer_range_check)]
        assert!(
            self.owns_stack_pointer(_sp),
            "Stack overflow detected in {:?}. Stack pointer: {:x}, Task stack range: {:x} ..= {:x}",
            self as *const Task,
            _sp,
            self.stack_range().0,
            self.stack_range().1
        );
    }

    pub(crate) fn set_up_stack_watchpoint(&self) {
        #[cfg(hw_task_overflow_detection)]
        unsafe {
            esp_hal::debugger::set_stack_watchpoint(self.stack_guard as usize);
        }
    }
}

pub(super) fn allocate_main_task(
    scheduler: &mut SchedulerState,
    stack: *mut [MaybeUninit<u32>],
    stack_guard_offset: usize,
    stack_guard_value: u32,
) {
    let cpu = Cpu::current();
    let current_cpu = cpu as usize;

    // Reset main task properties. The rest should be cleared when the task is deleted.
    scheduler.per_cpu[current_cpu].main_task.priority = Priority::ZERO;
    scheduler.per_cpu[current_cpu].main_task.state = TaskState::Ready;
    scheduler.per_cpu[current_cpu].main_task.stack = stack;
    scheduler.per_cpu[current_cpu]
        .main_task
        .in_run_or_wait_queue = false;
    scheduler.per_cpu[current_cpu].main_task.timer_queued = false;
    #[cfg(multi_core)]
    {
        scheduler.per_cpu[current_cpu].main_task.pinned_to = Some(cpu);
    }
    scheduler.per_cpu[current_cpu].main_task.thread_local = ThreadLocalData::new();

    scheduler.per_cpu[current_cpu]
        .main_task
        .set_up_stack_guard(stack_guard_offset, stack_guard_value);

    scheduler.per_cpu[current_cpu]
        .main_task
        .set_up_stack_watchpoint();

    // This is slightly questionable as we don't ensure SchedulerState is pinned, but it's always
    // part of a static object so taking the pointer is fine.
    let main_task_ptr = NonNull::from(&scheduler.per_cpu[current_cpu].main_task);

    scheduler.per_cpu[current_cpu]
        .main_task
        .cpu_context
        .set_tp(main_task_ptr.as_ptr() as u32);

    write_thread_pointer(main_task_ptr.as_ptr());

    debug!("Main task created: {:?}", main_task_ptr);

    #[cfg(feature = "rtos-trace")]
    rtos_trace::trace::task_new(main_task_ptr.rtos_trace_id());

    // The main task is already running, no need to add it to the ready queue.
    scheduler.all_tasks.push(main_task_ptr);
    #[cfg(multi_core)]
    scheduler.set_current_task(cpu, Some(main_task_ptr));
    scheduler
        .run_queue
        .mark_task_ready(&scheduler.per_cpu, scheduler.active_cores, main_task_ptr);
}

/// A handle to the current thread.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentThreadHandle {
    task: TaskPtr,
}

impl CurrentThreadHandle {
    /// Retrieves a handle to the current task.
    pub fn get() -> Self {
        Self {
            task: SCHEDULER.current_task(),
        }
    }

    /// Returns the name of the current thread.
    ///
    /// The main threads and the threads created by `esp-radio` have no name.
    pub fn name(self) -> Option<&'static str> {
        unsafe { self.task.as_ref().name }
    }

    /// Delays the current task for the specified duration.
    pub fn delay(self, duration: Duration) {
        self.delay_until(Instant::now() + duration);
    }

    /// Delays the current task until the specified deadline.
    pub fn delay_until(self, deadline: Instant) {
        SCHEDULER.sleep_until(deadline);
    }

    /// Sets the priority of the current task.
    pub fn set_priority(self, priority: usize) {
        let priority = Priority::new(priority);
        SCHEDULER.with(|scheduler| {
            let old = self.task.priority(&mut scheduler.run_queue);
            scheduler.set_priority(self.task, priority);

            // If we're dropping in priority, trigger a context switch in case another task can be
            // scheduled or time slicing needs to be started.
            if old > priority {
                yield_task();
            }
        });
    }
}

pub(super) fn schedule_task_deletion(task: Option<NonNull<Task>>) {
    trace!("schedule_task_deletion {:?}", task);
    if SCHEDULER.with(|scheduler| scheduler.schedule_task_deletion(task)) {
        loop {
            yield_task();
        }
    }
}

pub(crate) fn trigger_scheduler(run_scheduler: RunSchedulerOn) {
    match run_scheduler {
        RunSchedulerOn::DontRun => {}
        RunSchedulerOn::RunOnCore(_core) => {
            cfg_select! {
                multi_core => {
                    if _core == Cpu::current() {
                        yield_task()
                    } else {
                        schedule_other_core()
                    }
                }
                _ => yield_task(),
            }
        }
    }
}

#[inline]
#[cfg(multi_core)]
pub(crate) fn schedule_other_core() {
    use esp_hal::{
        interrupt::software::SoftwareInterrupt,
        peripherals::{FROM_CPU_INTR0, FROM_CPU_INTR1},
    };
    match Cpu::current() {
        Cpu::ProCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR1::steal() }).raise(),
        Cpu::AppCpu => SoftwareInterrupt::new(unsafe { FROM_CPU_INTR0::steal() }).raise(),
    }
}

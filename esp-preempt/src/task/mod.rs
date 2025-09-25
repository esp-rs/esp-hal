#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
pub(crate) mod arch_specific;

#[cfg(feature = "esp-radio")]
use core::ffi::c_void;
use core::{marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

#[cfg(feature = "alloc")]
use allocator_api2::alloc::{Allocator, Layout};
pub(crate) use arch_specific::*;
use esp_hal::{
    system::Cpu,
    time::{Duration, Instant},
};

#[cfg(feature = "alloc")]
use crate::InternalMemory;
#[cfg(feature = "esp-radio")]
use crate::semaphore::Semaphore;
use crate::{SCHEDULER, run_queue::RunQueue, scheduler::SchedulerState, wait_queue::WaitQueue};

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
    fn next(task: TaskPtr) -> Option<TaskPtr>;
    fn set_next(task: TaskPtr, next: Option<TaskPtr>);
}

macro_rules! task_list_item {
    ($struct:ident, $field:ident) => {
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
        }
    };
}

task_list_item!(TaskAllocListElement, alloc_list_item);
task_list_item!(TaskReadyQueueElement, ready_queue_item);
task_list_item!(TaskDeleteListElement, delete_list_item);
task_list_item!(TaskTimerQueueElement, timer_queue_item);

/// Extension trait for common task operations. These should be inherent methods but we can't
/// implement stuff for NonNull.
pub(crate) trait TaskExt {
    #[cfg(feature = "esp-radio")]
    fn resume(self);
    fn priority(self, _: &mut RunQueue) -> usize;
    fn set_priority(self, _: &mut RunQueue, new_pro: usize);
    fn state(self) -> TaskState;
    fn set_state(self, state: TaskState);
}

impl TaskExt for TaskPtr {
    #[cfg(feature = "esp-radio")]
    fn resume(self) {
        SCHEDULER.with(|scheduler| scheduler.resume_task(self))
    }

    fn priority(self, _: &mut RunQueue) -> usize {
        unsafe { self.as_ref().priority }
    }

    fn set_priority(mut self, run_queue: &mut RunQueue, new_priority: usize) {
        run_queue.remove(self);
        unsafe { self.as_mut().priority = new_priority };
    }

    fn state(self) -> TaskState {
        unsafe { self.as_ref().state }
    }

    fn set_state(mut self, state: TaskState) {
        trace!("Task {:?} state changed to {:?}", self, state);
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
        debug_assert!(E::next(task).is_none());
        E::set_next(task, self.head);
        self.head = Some(task);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        let popped = self.head.take();

        if let Some(task) = popped {
            self.head = E::next(task);
            E::set_next(task, None);
        }

        popped
    }

    pub fn remove(&mut self, task: TaskPtr) {
        // TODO: maybe this (and TaskQueue::remove) may prove too expensive.
        let mut list = core::mem::take(self);
        while let Some(popped) = list.pop() {
            if popped != task {
                self.push(popped);
            }
        }
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
        }

        popped
    }

    #[cfg(multi_core)]
    pub fn pop_if(&mut self, cond: impl Fn(&Task) -> bool) -> Option<TaskPtr> {
        let mut head = self.head.take();
        self.tail = None;

        let mut popped = None;
        while let Some(task) = head {
            head = E::next(task);
            E::set_next(task, None);
            if cond(unsafe { task.as_ref() }) {
                popped = Some(task);
                break;
            } else {
                self.push(task);
            }
        }

        while let Some(task) = head {
            head = E::next(task);
            E::set_next(task, None);
            self.push(task);
        }

        popped
    }

    pub fn remove(&mut self, task: TaskPtr) {
        let mut list = core::mem::take(self);
        while let Some(popped) = list.pop() {
            if popped != task {
                self.push(popped);
            }
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}

#[repr(C)]
pub(crate) struct Task {
    pub cpu_context: CpuContext,
    #[cfg(feature = "esp-radio")]
    pub thread_semaphore: Option<Semaphore>,
    pub state: TaskState,
    pub stack: *mut [MaybeUninit<u32>],
    pub priority: usize,
    #[cfg(multi_core)]
    pub pinned_to: Option<Cpu>,

    pub wakeup_at: u64,

    /// The current wait queue this task is in.
    pub(crate) current_queue: Option<NonNull<WaitQueue>>,

    // Lists a task can be in:
    /// The list of all allocated tasks
    pub alloc_list_item: TaskListItem,

    /// Either the RunQueue or the WaitQueue
    pub ready_queue_item: TaskListItem,

    /// The timer queue
    pub timer_queue_item: TaskListItem,

    /// The list of tasks scheduled for deletion
    pub delete_list_item: TaskListItem,

    /// Whether the task was allocated on the heap.
    #[cfg(feature = "alloc")]
    pub(crate) heap_allocated: bool,
}

const STACK_CANARY: u32 =
    const { esp_config::esp_config_int!(u32, "ESP_HAL_CONFIG_STACK_GUARD_VALUE") };

#[cfg(feature = "esp-radio")]
extern "C" fn task_wrapper(task_fn: extern "C" fn(*mut c_void), param: *mut c_void) {
    task_fn(param);
    schedule_task_deletion(core::ptr::null_mut());
}

impl Task {
    #[cfg(feature = "esp-radio")]
    pub(crate) fn new(
        name: &str,
        task_fn: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: usize,
        pinned_to: Option<Cpu>,
    ) -> Self {
        debug!(
            "task_create {} {:?}({:?}) stack_size = {} priority = {} pinned_to = {:?}",
            name, task_fn, param, task_stack_size, priority, pinned_to
        );

        // Make sure the stack guard doesn't eat into the stack size.
        let task_stack_size = task_stack_size + 4;

        // Make sure stack size is also aligned to 16 bytes.
        let task_stack_size = (task_stack_size & !0xF) + 16;

        let layout = unwrap!(
            Layout::from_size_align(task_stack_size, 16).ok(),
            "Cannot compute Layout for stack"
        );

        let stack = unwrap!(
            InternalMemory.allocate(layout).ok(),
            "Failed to allocate stack of {} bytes",
            layout.size()
        )
        .as_ptr();

        let stack_bottom = stack.cast::<MaybeUninit<u32>>();
        let stack_len_bytes = layout.size();
        unsafe { stack_bottom.write(MaybeUninit::new(STACK_CANARY)) };

        let stack_words = core::ptr::slice_from_raw_parts_mut(stack_bottom, stack_len_bytes / 4);
        let stack_top = unsafe { stack_bottom.add(stack_words.len()).cast() };

        Task {
            cpu_context: new_task_context(task_fn, param, stack_top),
            #[cfg(feature = "esp-radio")]
            thread_semaphore: None,
            state: TaskState::Ready,
            stack: stack_words,
            current_queue: None,
            priority,
            #[cfg(multi_core)]
            pinned_to,

            wakeup_at: 0,

            alloc_list_item: TaskListItem::None,
            ready_queue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,

            #[cfg(feature = "alloc")]
            heap_allocated: false,
        }
    }

    pub(crate) fn ensure_no_stack_overflow(&self) {
        assert_eq!(
            // This cast is safe to do from MaybeUninit<u32> because this is the word we've written
            // during initialization.
            unsafe { self.stack.cast::<u32>().read() },
            STACK_CANARY,
            "Stack overflow detected in {:?}",
            self as *const Task
        );
    }
}

impl Drop for Task {
    fn drop(&mut self) {
        debug!("Dropping task: {:?}", self as *mut Task);

        #[cfg(feature = "esp-radio")]
        let _ = self.thread_semaphore.take();

        #[cfg(feature = "alloc")]
        if self.heap_allocated {
            let layout = unwrap!(
                Layout::from_size_align(self.stack.len() * 4, 16).ok(),
                "Cannot compute Layout for stack"
            );
            unsafe { InternalMemory.deallocate(unwrap!(NonNull::new(self.stack.cast())), layout) };
        }
    }
}

pub(super) fn allocate_main_task(scheduler: &mut SchedulerState, stack: *mut [MaybeUninit<u32>]) {
    let cpu = Cpu::current();
    let current_cpu = cpu as usize;

    unsafe {
        stack
            .cast::<MaybeUninit<u32>>()
            .write(MaybeUninit::new(STACK_CANARY));
    }

    // Reset main task properties. The rest should be cleared when the task is deleted.
    scheduler.per_cpu[current_cpu].main_task.priority = 0;
    scheduler.per_cpu[current_cpu].main_task.state = TaskState::Ready;
    scheduler.per_cpu[current_cpu].main_task.stack = stack;
    #[cfg(multi_core)]
    {
        scheduler.per_cpu[current_cpu].main_task.pinned_to = Some(cpu);
    }

    debug_assert!(
        !scheduler.per_cpu[current_cpu].initialized,
        "Tried to allocate main task multiple times"
    );

    scheduler.per_cpu[current_cpu].initialized = true;

    // This is slightly questionable as we don't ensure SchedulerState is pinned, but it's always
    // part of a static object so taking the pointer is fine.
    let main_task_ptr = NonNull::from(&scheduler.per_cpu[current_cpu].main_task);
    debug!("Main task created: {:?}", main_task_ptr);

    // The main task is already running, no need to add it to the ready queue.
    scheduler.all_tasks.push(main_task_ptr);
    scheduler.per_cpu[current_cpu].current_task = Some(main_task_ptr);
    scheduler.run_queue.mark_task_ready(main_task_ptr);
}

pub(super) fn with_current_task<R>(mut cb: impl FnMut(&mut Task) -> R) -> R {
    SCHEDULER.with(|state| {
        cb(unsafe {
            let current_cpu = Cpu::current() as usize;
            unwrap!(state.per_cpu[current_cpu].current_task).as_mut()
        })
    })
}

pub(super) fn current_task() -> TaskPtr {
    with_current_task(|task| NonNull::from(task))
}

/// A handle to the current thread.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CurrentThreadHandle {
    _task: TaskPtr,
}

impl CurrentThreadHandle {
    /// Retrieves a handle to the current task.
    pub fn get() -> Self {
        Self {
            _task: current_task(),
        }
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
        let priority = priority.min(crate::run_queue::MaxPriority::MAX_PRIORITY);
        SCHEDULER.with(|state| {
            let current_cpu = Cpu::current() as usize;
            let current_task = unwrap!(state.per_cpu[current_cpu].current_task);
            let old = current_task.priority(&mut state.run_queue);
            current_task.set_priority(&mut state.run_queue, priority);
            if old > priority {
                crate::task::yield_task();
            }
        });
    }
}

#[cfg(feature = "esp-radio")]
pub(super) fn schedule_task_deletion(task: *mut Task) {
    trace!("schedule_task_deletion {:?}", task);
    SCHEDULER.with(|scheduler| {
        if scheduler.schedule_task_deletion(task) {
            yield_task();
        }
    });
}

#[inline]
#[cfg(multi_core)]
pub(crate) fn schedule_other_core() {
    use esp_hal::interrupt::software::SoftwareInterrupt;
    match Cpu::current() {
        Cpu::ProCpu => unsafe { SoftwareInterrupt::<'static, 1>::steal() }.raise(),
        Cpu::AppCpu => unsafe { SoftwareInterrupt::<'static, 0>::steal() }.raise(),
    }

    // It takes a bit for the software interrupt to be serviced, but since it's happening on the
    // other core, we don't need to wait.
}

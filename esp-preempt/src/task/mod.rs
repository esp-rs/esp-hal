#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
pub(crate) mod arch_specific;

use core::{ffi::c_void, marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

use allocator_api2::boxed::Box;
pub(crate) use arch_specific::*;
use esp_radio_preempt_driver::semaphore::{SemaphoreHandle, SemaphorePtr};

use crate::{
    InternalMemory,
    SCHEDULER,
    run_queue::RunQueue,
    scheduler::SchedulerState,
    wait_queue::WaitQueue,
};

#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum TaskState {
    Ready,
    Sleeping,
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
    fn resume(self);
    fn priority(self, _: &mut RunQueue) -> usize;
    fn set_priority(self, _: &mut RunQueue, new_pro: usize);
    fn state(self) -> TaskState;
    fn set_state(self, state: TaskState);
}

impl TaskExt for TaskPtr {
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
    pub thread_semaphore: Option<SemaphorePtr>,
    pub state: TaskState,
    pub stack: *mut [MaybeUninit<u32>],
    pub priority: usize,

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
    pub(crate) heap_allocated: bool,
}

const STACK_CANARY: u32 =
    const { esp_config::esp_config_int!(u32, "ESP_HAL_CONFIG_STACK_GUARD_VALUE") };

impl Task {
    pub(crate) fn new(
        task_fn: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: usize,
    ) -> Self {
        trace!(
            "task_create {:?}({:?}) stack_size = {} priority = {}",
            task_fn, param, task_stack_size, priority
        );

        let task_stack_size_words = task_stack_size / 4;
        let mut stack = Box::<[u32], _>::new_uninit_slice_in(task_stack_size_words, InternalMemory);
        stack[0] = MaybeUninit::new(STACK_CANARY);

        let stack = Box::leak(stack) as *mut [MaybeUninit<u32>];
        let stack_top = unsafe { stack.cast::<u32>().add(task_stack_size_words).cast() };

        Task {
            cpu_context: new_task_context(task_fn, param, stack_top),
            thread_semaphore: None,
            state: TaskState::Ready,
            stack,
            current_queue: None,
            priority,

            wakeup_at: 0,

            alloc_list_item: TaskListItem::None,
            ready_queue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,

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
        if let Some(sem) = self.thread_semaphore {
            let sem = unsafe { SemaphoreHandle::from_ptr(sem) };
            core::mem::drop(sem)
        }
    }
}

// This context will be filled out by the first context switch.
// We allocate the main task statically, because there is always a main task. If deleted, we simply
// don't deallocate this.
pub(crate) static mut MAIN_TASK: Task = Task {
    cpu_context: CpuContext::new(),
    thread_semaphore: None,
    state: TaskState::Ready,
    stack: core::ptr::slice_from_raw_parts_mut(core::ptr::null_mut(), 0),
    current_queue: None,
    priority: 0,

    wakeup_at: 0,

    alloc_list_item: TaskListItem::None,
    ready_queue_item: TaskListItem::None,
    timer_queue_item: TaskListItem::None,
    delete_list_item: TaskListItem::None,

    heap_allocated: false,
};

pub(super) fn allocate_main_task(scheduler: &mut SchedulerState, stack: *mut [MaybeUninit<u32>]) {
    let mut main_task_ptr = unwrap!(NonNull::new(&raw mut MAIN_TASK));
    debug!("Main task created: {:?}", main_task_ptr);

    unsafe {
        stack
            .cast::<MaybeUninit<u32>>()
            .write(MaybeUninit::new(STACK_CANARY));

        let main_task = main_task_ptr.as_mut();

        // Reset main task properties. The rest should be cleared when the task is deleted.
        main_task.priority = 0;
        main_task.state = TaskState::Ready;
        main_task.stack = stack;
    }

    debug_assert!(
        scheduler.current_task.is_none(),
        "Tried to allocate main task multiple times"
    );

    // The main task is already running, no need to add it to the ready queue.
    scheduler.all_tasks.push(main_task_ptr);
    scheduler.current_task = Some(main_task_ptr);
    scheduler.run_queue.mark_task_ready(main_task_ptr);
}

pub(super) fn with_current_task<R>(mut cb: impl FnMut(&mut Task) -> R) -> R {
    SCHEDULER.with(|state| cb(unsafe { unwrap!(state.current_task).as_mut() }))
}

pub(super) fn current_task() -> TaskPtr {
    with_current_task(|task| NonNull::from(task))
}

pub(super) fn schedule_task_deletion(task: *mut Task) {
    trace!("schedule_task_deletion {:?}", task);
    let deleting_current = SCHEDULER.with(|state| state.schedule_task_deletion(task));

    // Tasks are deleted during context switches, so we need to yield if we are
    // deleting the current task.
    if deleting_current {
        SCHEDULER.with(|scheduler| {
            // We won't be re-scheduled.
            scheduler.event.set_blocked();
            yield_task();
        });
        unreachable!();
    }
}

#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
pub(crate) mod arch_specific;

use core::{ffi::c_void, marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

use allocator_api2::boxed::Box;
#[cfg(riscv)]
use arch_specific::Registers;
pub(crate) use arch_specific::*;
#[cfg(xtensa)]
use esp_hal::trapframe::TrapFrame;
use esp_radio_preempt_driver::semaphore::{SemaphoreHandle, SemaphorePtr};

use crate::{InternalMemory, SCHEDULER, run_queue::RunQueue, wait_queue::WaitQueue};

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
        unsafe { self.as_ref().priority as usize }
    }

    fn set_priority(mut self, run_queue: &mut RunQueue, new_pro: usize) {
        run_queue.remove(self);
        unsafe { self.as_mut().priority = new_pro as u32 };
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
    #[cfg(riscv)]
    pub trap_frame: Registers,
    #[cfg(xtensa)]
    pub trap_frame: TrapFrame,
    pub thread_semaphore: Option<SemaphorePtr>,
    pub state: TaskState,
    pub _allocated_stack: Box<[MaybeUninit<u32>], InternalMemory>,
    pub priority: u32,

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
}

const STACK_CANARY: u32 = 0xDEEDBAAD;

impl Task {
    pub(crate) fn new(
        task_fn: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
        priority: u32,
    ) -> Self {
        trace!(
            "task_create {:?}({:?}) stack_size = {} priority = {}",
            task_fn, param, task_stack_size, priority
        );

        let task_stack_size_words = task_stack_size / 4;
        let mut stack = Box::<[u32], _>::new_uninit_slice_in(task_stack_size_words, InternalMemory);

        let stack_top = unsafe { stack.as_mut_ptr().add(task_stack_size_words).cast() };

        stack[0] = MaybeUninit::new(STACK_CANARY);

        Task {
            trap_frame: new_task_context(task_fn, param, stack_top),
            thread_semaphore: None,
            state: TaskState::Ready,
            _allocated_stack: stack,
            current_queue: None,
            priority,

            wakeup_at: 0,

            alloc_list_item: TaskListItem::None,
            ready_queue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,
        }
    }

    pub(crate) fn ensure_no_stack_overflow(&self) {
        if self._allocated_stack.is_empty() {
            return;
        }

        assert_eq!(
            unsafe { self._allocated_stack[0].assume_init() },
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

pub(super) fn allocate_main_task() {
    // This context will be filled out by the first context switch.
    let task = Box::new_in(
        Task {
            #[cfg(riscv)]
            trap_frame: Registers::default(),
            #[cfg(xtensa)]
            trap_frame: TrapFrame::default(),
            thread_semaphore: None,
            state: TaskState::Ready,
            _allocated_stack: Box::<[u32], _>::new_uninit_slice_in(0, InternalMemory),
            current_queue: None,
            priority: 1,

            wakeup_at: 0,

            alloc_list_item: TaskListItem::None,
            ready_queue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,
        },
        InternalMemory,
    );
    let main_task_ptr = NonNull::from(Box::leak(task));
    debug!("Main task created: {:?}", main_task_ptr);

    SCHEDULER.with(|state| {
        debug_assert!(
            state.current_task.is_none(),
            "Tried to allocate main task multiple times"
        );

        // The main task is already running, no need to add it to the ready queue.
        state.all_tasks.push(main_task_ptr);
        state.current_task = Some(main_task_ptr);
        state.run_queue.mark_task_ready(main_task_ptr);
    })
}

pub(crate) fn spawn_idle_task() {
    let ptr = SCHEDULER.create_task(idle_task, core::ptr::null_mut(), 4096, 0);
    debug!("Idle task created: {:?}", ptr);
}

pub(crate) extern "C" fn idle_task(_: *mut c_void) {
    loop {
        // TODO: make this configurable.
        #[cfg(xtensa)]
        unsafe {
            core::arch::asm!("waiti 0");
        }
        #[cfg(riscv)]
        unsafe {
            core::arch::asm!("wfi");
        }
    }
}

pub(super) fn delete_all_tasks() {
    trace!("delete_all_tasks");
    let mut all_tasks = SCHEDULER.with(|state| {
        // Since we delete all tasks, we walk through the allocation list - we just need to clear
        // the lists.
        state.to_delete = TaskList::new();
        state.run_queue = RunQueue::new();

        // Clear the current task.
        state.current_task = None;

        // Take the allocation list
        core::mem::take(&mut state.all_tasks)
    });

    while let Some(task) = all_tasks.pop() {
        unsafe {
            let task = Box::from_raw_in(task.as_ptr(), InternalMemory);
            core::mem::drop(task);
        }
    }
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

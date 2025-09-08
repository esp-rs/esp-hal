#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
pub(crate) mod arch_specific;

use core::{ffi::c_void, marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

use allocator_api2::boxed::Box;
#[cfg(riscv)]
use arch_specific::Registers;
pub(crate) use arch_specific::*;
use esp_hal::time::Instant;
#[cfg(xtensa)]
use esp_hal::trapframe::TrapFrame;
use esp_radio_preempt_driver::semaphore::{SemaphoreHandle, SemaphorePtr};

use crate::{InternalMemory, SCHEDULER, run_queue::RunQueue};

#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum TaskState {
    Ready,
    Sleeping,
}

pub(crate) type TaskPtr = NonNull<Context>;
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
task_list_item!(TaskReadyQueueElement, ready_quue_item);
task_list_item!(TaskDeleteListElement, delete_list_item);
task_list_item!(TaskTimerQueueElement, timer_queue_item);

/// Extension trait for common task operations. These should be inherent methods but we can't
/// implement stuff for NonNull.
pub(crate) trait TaskExt {
    fn sleep_until(self, wakeup_time: Instant);
}

impl TaskExt for TaskPtr {
    fn sleep_until(self, wakeup_time: Instant) {
        SCHEDULER.with(|scheduler| scheduler.sleep_until(self, wakeup_time))
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
        E::set_next(task, self.head);
        self.head = Some(task);
    }

    pub fn pop(&mut self) -> Option<TaskPtr> {
        let popped = self.head.take();

        if let Some(task) = popped {
            self.head = E::next(task);
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
        E::set_next(task, None);
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
pub(crate) struct Context {
    #[cfg(riscv)]
    pub trap_frame: Registers,
    #[cfg(xtensa)]
    pub trap_frame: TrapFrame,
    pub thread_semaphore: Option<SemaphorePtr>,
    pub state: TaskState,
    pub _allocated_stack: Box<[MaybeUninit<u8>], InternalMemory>,

    pub wakeup_at: u64,

    // Lists a task can be in:
    /// The list of all allocated tasks
    pub alloc_list_item: TaskListItem,

    /// The list of ready tasks
    pub ready_quue_item: TaskListItem,

    /// The timer queue
    pub timer_queue_item: TaskListItem,

    /// The list of tasks scheduled for deletion
    pub delete_list_item: TaskListItem,
}

impl Context {
    pub(crate) fn new(
        task_fn: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
    ) -> Self {
        trace!("task_create {:?} {:?} {}", task_fn, param, task_stack_size);

        let mut stack = Box::<[u8], _>::new_uninit_slice_in(task_stack_size, InternalMemory);

        let stack_top = unsafe { stack.as_mut_ptr().add(task_stack_size).cast() };

        Context {
            trap_frame: new_task_context(task_fn, param, stack_top),
            thread_semaphore: None,
            state: TaskState::Ready,
            _allocated_stack: stack,

            wakeup_at: 0,

            alloc_list_item: TaskListItem::None,
            ready_quue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,
        }
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        if let Some(sem) = self.thread_semaphore {
            let sem = unsafe { SemaphoreHandle::from_ptr(sem) };
            core::mem::drop(sem)
        }
    }
}

pub(super) fn allocate_main_task() {
    // This context will be filled out by the first context switch.
    let task = Box::new_in(
        Context {
            #[cfg(riscv)]
            trap_frame: Registers::default(),
            #[cfg(xtensa)]
            trap_frame: TrapFrame::default(),
            thread_semaphore: None,
            state: TaskState::Ready,
            _allocated_stack: Box::<[u8], _>::new_uninit_slice_in(0, InternalMemory),

            wakeup_at: 0,

            alloc_list_item: TaskListItem::None,
            ready_quue_item: TaskListItem::None,
            timer_queue_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,
        },
        InternalMemory,
    );
    let main_task_ptr = NonNull::from(Box::leak(task));

    SCHEDULER.with(|state| {
        debug_assert!(
            state.current_task.is_none(),
            "Tried to allocate main task multiple times"
        );

        // The main task is already running, no need to add it to the ready queue.
        state.all_tasks.push(main_task_ptr);
        state.current_task = Some(main_task_ptr);
    })
}

pub(super) fn delete_all_tasks() {
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

pub(super) fn with_current_task<R>(mut cb: impl FnMut(&mut Context) -> R) -> R {
    SCHEDULER.with(|state| cb(unsafe { unwrap!(state.current_task).as_mut() }))
}

pub(super) fn current_task() -> TaskPtr {
    with_current_task(|task| NonNull::from(task))
}

pub(super) fn schedule_task_deletion(task: *mut Context) {
    let deleting_current = SCHEDULER.with(|state| state.schedule_task_deletion(task));

    // Tasks are deleted during context switches, so we need to yield if we are
    // deleting the current task.
    if deleting_current {
        loop {
            SCHEDULER.yield_task();
        }
    }
}

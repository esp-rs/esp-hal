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

use crate::{InternalMemory, SCHEDULER_STATE, task, timer};

#[derive(Clone, Copy)]
pub(crate) enum TaskState {
    Ready,
}

impl TaskState {
    pub fn is_ready(self) -> bool {
        matches!(self, Self::Ready)
    }
}

pub(crate) type TaskPtr = NonNull<Context>;
pub(crate) type TaskListItem = Option<TaskPtr>;

/// An abstraction that allows the task to contain multiple different queue pointers.
pub(crate) trait TaskListElement {
    fn next(task: TaskPtr) -> Option<TaskPtr>;
    fn set_next(task: TaskPtr, next: Option<TaskPtr>);
}

pub(crate) struct TaskAllocListElement;
impl TaskListElement for TaskAllocListElement {
    fn next(task: TaskPtr) -> Option<TaskPtr> {
        unsafe { task.as_ref().alloc_list_item }
    }

    fn set_next(mut task: TaskPtr, next: Option<TaskPtr>) {
        unsafe {
            task.as_mut().alloc_list_item = next;
        }
    }
}

pub(crate) struct TaskReadyListElement;
impl TaskListElement for TaskReadyListElement {
    fn next(task: TaskPtr) -> Option<TaskPtr> {
        unsafe { task.as_ref().ready_list_item }
    }

    fn set_next(mut task: TaskPtr, next: Option<TaskPtr>) {
        unsafe {
            task.as_mut().ready_list_item = next;
        }
    }
}

pub(crate) struct TaskDeleteListElement;
impl TaskListElement for TaskDeleteListElement {
    fn next(task: TaskPtr) -> Option<TaskPtr> {
        unsafe { task.as_ref().delete_list_item }
    }

    fn set_next(mut task: TaskPtr, next: Option<TaskPtr>) {
        unsafe {
            task.as_mut().delete_list_item = next;
        }
    }
}

/// A singly linked list of tasks.
///
/// Use this where you don't care about the order of list elements.
///
/// The `E` type parameter is used to access the data in the task object that belongs to this list.
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

    // Lists a task can be in:
    /// The list of all allocated tasks
    pub alloc_list_item: TaskListItem,

    /// The list of ready tasks
    pub ready_list_item: TaskListItem,

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
            trap_frame: task::new_task_context(task_fn, param, stack_top),
            thread_semaphore: None,
            state: TaskState::Ready,
            _allocated_stack: stack,

            alloc_list_item: TaskListItem::None,
            ready_list_item: TaskListItem::None,
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
    let context = Box::new_in(
        Context {
            #[cfg(riscv)]
            trap_frame: Registers::default(),
            #[cfg(xtensa)]
            trap_frame: TrapFrame::default(),
            thread_semaphore: None,
            state: TaskState::Ready,
            _allocated_stack: Box::<[u8], _>::new_uninit_slice_in(0, InternalMemory),

            alloc_list_item: TaskListItem::None,
            ready_list_item: TaskListItem::None,
            delete_list_item: TaskListItem::None,
        },
        InternalMemory,
    );

    SCHEDULER_STATE.with(|state| {
        debug_assert!(
            state.current_task.is_none(),
            "Tried to allocate main task multiple times"
        );

        let main_task = NonNull::from(Box::leak(context));

        state.all_tasks.push(main_task);
        state.current_task = Some(main_task);

        // The first task loops back to itself.
        TaskReadyListElement::set_next(main_task, Some(main_task));
    })
}

pub(super) fn delete_all_tasks() {
    let mut all_tasks = SCHEDULER_STATE.with(|state| {
        // Since we delete all tasks, we walk through the allocation list - we just need to clear
        // the delete list.
        state.to_delete = TaskList::new();

        // Clear the current task.
        state.current_task = None;

        // Take the allocation list
        core::mem::replace(&mut state.all_tasks, TaskList::new())
    });

    while let Some(task) = all_tasks.pop() {
        unsafe {
            core::ptr::drop_in_place(task.as_ptr());
        }
    }
}

pub(super) fn with_current_task<R>(mut cb: impl FnMut(&mut Context) -> R) -> R {
    SCHEDULER_STATE.with(|state| cb(unsafe { unwrap!(state.current_task).as_mut() }))
}

pub(super) fn current_task() -> *mut Context {
    with_current_task(|task| task as *mut Context)
}

pub(super) fn schedule_task_deletion(task: *mut Context) {
    let deleting_current = SCHEDULER_STATE.with(|state| state.schedule_task_deletion(task));

    // Tasks are deleted during context switches, so we need to yield if we are
    // deleting the current task.
    if deleting_current {
        loop {
            timer::yield_task();
        }
    }
}

#[cfg(riscv)]
pub(crate) fn task_switch() {
    SCHEDULER_STATE.with(|state| state.switch_task());
}

#[cfg(xtensa)]
pub(crate) fn task_switch(trap_frame: &mut TrapFrame) {
    SCHEDULER_STATE.with(|state| state.switch_task(trap_frame));
}

#[cfg_attr(target_arch = "riscv32", path = "riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "xtensa.rs")]
mod arch_specific;

use core::{ffi::c_void, mem::MaybeUninit};

use allocator_api2::boxed::Box;
pub(crate) use arch_specific::*;
use esp_hal::trapframe::TrapFrame;

use crate::{InternalMemory, SCHEDULER_STATE, task, timer};

pub(crate) struct Context {
    trap_frame: TrapFrame,
    pub thread_semaphore: u32,
    pub next: *mut Context,
    pub _allocated_stack: Box<[MaybeUninit<u8>], InternalMemory>,
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
            thread_semaphore: 0,
            next: core::ptr::null_mut(),
            _allocated_stack: stack,
        }
    }
}

pub(super) fn allocate_main_task() {
    // This context will be filled out by the first context switch.
    let context = Box::new_in(
        Context {
            trap_frame: TrapFrame::default(),
            thread_semaphore: 0,
            next: core::ptr::null_mut(),
            _allocated_stack: Box::<[u8], _>::new_uninit_slice_in(0, InternalMemory),
        },
        InternalMemory,
    );

    let context_ptr = Box::into_raw(context);
    unsafe {
        // The first task loops back to itself.
        (*context_ptr).next = context_ptr;
    }

    SCHEDULER_STATE.with(|state| {
        debug_assert!(
            state.current_task.is_null(),
            "Tried to allocate main task multiple times"
        );
        state.current_task = context_ptr;
    })
}

pub(super) fn delete_all_tasks() {
    let first_task = SCHEDULER_STATE.with(|state| {
        // Remove all tasks from the list. We will drop them outside of the critical
        // section.
        core::mem::take(&mut state.current_task)
    });

    if first_task.is_null() {
        return;
    }

    let mut task_to_delete = first_task;

    loop {
        let next_task = unsafe {
            // SAFETY: Tasks are in a circular linked list. We are guaranteed that the next
            // task is a valid pointer, or the first task that may already have been
            // deleted. In the second case, we will not move on to the next
            // iteration, so the loop will not try to free a task twice.
            let next_task = (*task_to_delete).next;
            core::ptr::drop_in_place(task_to_delete);
            next_task
        };

        if core::ptr::eq(next_task, first_task) {
            break;
        }

        task_to_delete = next_task;
    }
}

pub(super) fn current_task() -> *mut Context {
    SCHEDULER_STATE.with(|state| state.current_task)
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

pub(crate) fn task_switch(trap_frame: &mut TrapFrame) {
    SCHEDULER_STATE.with(|state| state.switch_task(trap_frame));
}

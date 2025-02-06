#[cfg_attr(target_arch = "riscv32", path = "preempt_riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "preempt_xtensa.rs")]
mod arch_specific;
pub mod timer;

use core::mem::size_of;

use arch_specific::*;
use esp_hal::sync::Locked;
use esp_wifi_sys::include::malloc;
use timer::{disable_multitasking, disable_timer, setup_multitasking, setup_timer};
//
pub(crate) use {arch_specific::task_create, timer::yield_task};

use crate::{compat::malloc::free, hal::trapframe::TrapFrame, memory_fence::memory_fence};

#[repr(transparent)]
struct ContextWrapper(*mut Context);

unsafe impl Send for ContextWrapper {}

static CTX_NOW: Locked<ContextWrapper> = Locked::new(ContextWrapper(core::ptr::null_mut()));

static mut SCHEDULED_TASK_TO_DELETE: *mut Context = core::ptr::null_mut();

pub(crate) fn setup(timer: crate::TimeBase) {
    // allocate the main task
    allocate_main_task();
    setup_timer(timer);
    setup_multitasking();
}

pub(crate) fn disable() {
    disable_timer();
    disable_multitasking();
}

fn allocate_main_task() -> *mut Context {
    CTX_NOW.with(|ctx_now| unsafe {
        if !ctx_now.0.is_null() {
            panic!("Tried to allocate main task multiple times");
        }

        let ptr = malloc(size_of::<Context>() as u32) as *mut Context;
        core::ptr::write(ptr, Context::new());
        (*ptr).next = ptr;
        ctx_now.0 = ptr;
        ptr
    })
}

fn allocate_task() -> *mut Context {
    CTX_NOW.with(|ctx_now| unsafe {
        if ctx_now.0.is_null() {
            panic!("Called `allocate_task` before allocating main task");
        }

        let ptr = malloc(size_of::<Context>() as u32) as *mut Context;
        core::ptr::write(ptr, Context::new());
        (*ptr).next = (*ctx_now.0).next;
        (*ctx_now.0).next = ptr;
        ptr
    })
}

fn next_task() {
    CTX_NOW.with(|ctx_now| unsafe {
        ctx_now.0 = (*ctx_now.0).next;
    });
}

/// Delete the given task.
///
/// This will also free the memory (stack and context) allocated for it.
pub(crate) fn delete_task(task: *mut Context) {
    CTX_NOW.with(|ctx_now| unsafe {
        let mut ptr = ctx_now.0;
        let initial = ptr;
        loop {
            if (*ptr).next == task {
                (*ptr).next = (*((*ptr).next)).next;

                free((*task).allocated_stack as *mut u8);
                free(task as *mut u8);
                break;
            }

            ptr = (*ptr).next;

            if ptr == initial {
                break;
            }
        }

        memory_fence();
    });
}

pub(crate) fn delete_all_tasks() {
    CTX_NOW.with(|ctx_now| unsafe {
        let current_task = ctx_now.0;

        if current_task.is_null() {
            return;
        }

        let mut task_to_delete = current_task;

        loop {
            let next_task = (*task_to_delete).next;

            free((*task_to_delete).allocated_stack as *mut u8);
            free(task_to_delete as *mut u8);

            if next_task == current_task {
                break;
            }

            task_to_delete = next_task;
        }

        ctx_now.0 = core::ptr::null_mut();

        memory_fence();
    });
}

pub(crate) fn current_task() -> *mut Context {
    CTX_NOW.with(|ctx_now| ctx_now.0)
}

pub(crate) fn schedule_task_deletion(task: *mut Context) {
    unsafe {
        SCHEDULED_TASK_TO_DELETE = task;
    }

    if task == current_task() {
        loop {
            yield_task();
        }
    }
}

pub(crate) fn task_switch(trap_frame: &mut TrapFrame) {
    save_task_context(current_task(), trap_frame);

    unsafe {
        if !SCHEDULED_TASK_TO_DELETE.is_null() {
            delete_task(SCHEDULED_TASK_TO_DELETE);
            SCHEDULED_TASK_TO_DELETE = core::ptr::null_mut();
        }
    }

    next_task();
    restore_task_context(current_task(), trap_frame);
}

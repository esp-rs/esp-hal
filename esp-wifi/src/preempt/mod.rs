#[cfg_attr(target_arch = "riscv32", path = "preempt_riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "preempt_xtensa.rs")]
pub mod arch_specific;

use arch_specific::*;
use esp_wifi_sys::include::malloc;

use crate::{compat::malloc::free, hal::trapframe::TrapFrame, memory_fence::memory_fence};

static mut CTX_NOW: *mut Context = core::ptr::null_mut();

static mut SCHEDULED_TASK_TO_DELETE: *mut Context = core::ptr::null_mut();

pub fn allocate_main_task() -> *mut Context {
    critical_section::with(|_| unsafe {
        if !CTX_NOW.is_null() {
            panic!("Tried to allocate main task multiple times");
        }

        let ptr = malloc(size_of::<Context>() as u32) as *mut Context;
        core::ptr::write_bytes(ptr, 0, 1);
        (*ptr).next = ptr;
        CTX_NOW = ptr;
        ptr
    })
}

fn allocate_task() -> *mut Context {
    critical_section::with(|_| unsafe {
        if CTX_NOW.is_null() {
            panic!("Called `allocate_task` before allocating main task");
        }

        let ptr = malloc(size_of::<Context>() as u32) as *mut Context;
        core::ptr::write_bytes(ptr, 0, 1);
        (*ptr).next = (*CTX_NOW).next;
        (*CTX_NOW).next = ptr;
        ptr
    })
}

fn next_task() {
    unsafe {
        CTX_NOW = (*CTX_NOW).next;
    }
}

/// Delete the given task
///
/// This will also free the memory allocated for it.
fn delete_task(task: *mut Context) {
    critical_section::with(|_| unsafe {
        let mut ptr = CTX_NOW;
        loop {
            if (*ptr).next == task {
                (*ptr).next = (*(*ptr).next).next;

                free((*task).allocated_stack as *mut u8);
                free(task as *mut u8);
                break;
            }

            ptr = (*ptr).next;

            if ptr == CTX_NOW {
                break;
            }
        }

        memory_fence();
    });
}

pub fn current_task() -> *mut Context {
    unsafe { CTX_NOW }
}

#[cfg(feature = "wifi")]
pub fn schedule_task_deletion(task: *mut Context) {
    use crate::timer::yield_task;

    unsafe {
        SCHEDULED_TASK_TO_DELETE = task;
    }

    if task == current_task() {
        loop {
            yield_task();
        }
    }
}

pub fn task_switch(trap_frame: &mut TrapFrame) {
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

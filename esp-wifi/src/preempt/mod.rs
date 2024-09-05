#[cfg_attr(target_arch = "riscv32", path = "preempt_riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "preempt_xtensa.rs")]
pub mod arch_specific;

use core::{cell::RefCell, mem::size_of};

use arch_specific::*;
use critical_section::Mutex;
use esp_wifi_sys::include::malloc;

use crate::{compat::malloc::free, hal::trapframe::TrapFrame, memory_fence::memory_fence};

static mut CTX_NOW: Mutex<RefCell<*mut Context>> = Mutex::new(RefCell::new(core::ptr::null_mut()));

static mut SCHEDULED_TASK_TO_DELETE: *mut Context = core::ptr::null_mut();

pub fn allocate_main_task() -> *mut Context {
    critical_section::with(|cs| unsafe {
        let mut ctx_now = CTX_NOW.borrow_ref_mut(cs);
        if !(*ctx_now).is_null() {
            panic!("Tried to allocate main task multiple times");
        }

        let ptr = malloc(size_of::<Context>() as u32) as *mut Context;
        core::ptr::write_bytes(ptr, 0, 1);
        (*ptr).next = ptr;
        *ctx_now = ptr;
        ptr
    })
}

fn allocate_task() -> *mut Context {
    critical_section::with(|cs| unsafe {
        let mut ctx_now = CTX_NOW.borrow_ref_mut(cs);
        if (*ctx_now).is_null() {
            panic!("Called `allocate_task` before allocating main task");
        }

        let ptr = malloc(size_of::<Context>() as u32) as *mut Context;
        core::ptr::write_bytes(ptr, 0, 1);
        (*ptr).next = (**ctx_now).next;
        (**ctx_now).next = ptr;
        ptr
    })
}

fn next_task() {
    critical_section::with(|cs| unsafe {
        let mut ctx_now = CTX_NOW.borrow_ref_mut(cs);
        *ctx_now = (**ctx_now).next;
    });
}

/// Delete the given task.
///
/// This will also free the memory (stack and context) allocated for it.
fn delete_task(task: *mut Context) {
    critical_section::with(|cs| unsafe {
        let mut ptr = *CTX_NOW.borrow_ref_mut(cs);
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

pub fn current_task() -> *mut Context {
    critical_section::with(|cs| unsafe { *CTX_NOW.borrow_ref(cs) })
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

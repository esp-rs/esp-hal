use esp_wifi_sys::c_types;

use super::*;
use crate::hal::trapframe::TrapFrame;

#[derive(Debug, Clone, Copy)]
pub struct Context {
    trap_frame: TrapFrame,
    pub thread_semaphore: u32,
    pub next: *mut Context,
    pub allocated_stack: *const u8,
}

pub fn task_create(
    task: extern "C" fn(*mut c_types::c_void),
    param: *mut c_types::c_void,
    task_stack_size: usize,
) -> *mut Context {
    trace!("task_create {:?} {:?} {}", task, param, task_stack_size);
    unsafe {
        let ctx = allocate_task();

        (*ctx).trap_frame.PC = task as usize as u32;
        (*ctx).trap_frame.A6 = param as usize as u32;

        let stack = malloc(task_stack_size as u32);
        (*ctx).allocated_stack = stack.cast();

        // stack must be aligned by 16
        let task_stack_ptr = stack as usize + task_stack_size;
        let stack_ptr = task_stack_ptr - (task_stack_ptr % 0x10);
        (*ctx).trap_frame.A1 = stack_ptr as u32;

        (*ctx).trap_frame.PS = 0x00040000 | (1 & 3) << 16; // For windowed ABI set WOE and CALLINC (pretend task was 'call4'd).

        (*ctx).trap_frame.A0 = 0;

        *((task_stack_ptr - 4) as *mut u32) = 0;
        *((task_stack_ptr - 8) as *mut u32) = 0;
        *((task_stack_ptr - 12) as *mut u32) = stack_ptr as u32;
        *((task_stack_ptr - 16) as *mut u32) = 0;

        ctx
    }
}

pub fn restore_task_context(ctx: *mut Context, trap_frame: &mut TrapFrame) {
    unsafe {
        *trap_frame = (*ctx).trap_frame;
    }
}

pub fn save_task_context(ctx: *mut Context, trap_frame: &TrapFrame) {
    unsafe {
        (*ctx).trap_frame = *trap_frame;
    }
}

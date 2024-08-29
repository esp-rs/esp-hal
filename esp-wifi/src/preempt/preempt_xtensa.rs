use core::ptr::addr_of;

use super::*;
use crate::hal::trapframe::TrapFrame;

#[derive(Debug, Clone, Copy)]
pub struct TaskContext {
    trap_frame: TrapFrame,
}

static mut CTX_TASKS: [TaskContext; MAX_TASK] = [TaskContext {
    trap_frame: TrapFrame::new(),
}; MAX_TASK];

pub fn task_create(task: extern "C" fn()) {
    unsafe {
        let i = allocate_task();

        CTX_TASKS[i].trap_frame.PC = task as usize as u32;

        let task_stack_size = TASK_STACK_SIZE[i];

        // stack must be aligned by 16
        let task_stack_ptr =
            (addr_of!(TASK_STACK) as *const _ as usize + (task_stack_size * i) + task_stack_size
                - 4) as u32;
        let stack_ptr = task_stack_ptr - (task_stack_ptr % 0x10);
        CTX_TASKS[i].trap_frame.A1 = stack_ptr;

        CTX_TASKS[i].trap_frame.PS = 0x00040000 | (1 & 3) << 16; // For windowed ABI set WOE and CALLINC (pretend task was 'call4'd).

        CTX_TASKS[i].trap_frame.A0 = 0;

        *((task_stack_ptr - 4) as *mut u32) = 0;
        *((task_stack_ptr - 8) as *mut u32) = 0;
        *((task_stack_ptr - 12) as *mut u32) = stack_ptr;
        *((task_stack_ptr - 16) as *mut u32) = 0;
    }
}

fn restore_task_context(id: usize, trap_frame: &mut TrapFrame) {
    unsafe {
        *trap_frame = CTX_TASKS[id].trap_frame;
    }
}

fn save_task_context(id: usize, trap_frame: &TrapFrame) {
    unsafe {
        CTX_TASKS[id].trap_frame = *trap_frame;
    }
}

pub fn task_switch(trap_frame: &mut TrapFrame) {
    save_task_context(current_task(), trap_frame);
    next_task();
    restore_task_context(current_task(), trap_frame);

    // debug aid! remove when not needed anymore!!!!!
    // static mut CNT: u32 = 0;
    // if CTX_NOW == 0 {
    //     if CNT < 2_000 {
    //         CNT += 1;
    //     } else {
    //         CNT = 0;
    //         info!("@@@ Task {} {:?} ", 1, CTX_TASKS[1].trap_frame.PC);
    //     }
    // }
}

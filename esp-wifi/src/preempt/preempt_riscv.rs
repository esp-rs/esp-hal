use core::ptr::addr_of;

use super::*;
use crate::hal::interrupt::TrapFrame;

#[derive(Debug, Default, Clone, Copy)]
pub struct Context {
    trap_frame: TrapFrame,
    _running: bool,
}

static mut CTX_TASKS: [Context; MAX_TASK] = [Context {
    trap_frame: TrapFrame {
        ra: 0,
        t0: 0,
        t1: 0,
        t2: 0,
        t3: 0,
        t4: 0,
        t5: 0,
        t6: 0,
        a0: 0,
        a1: 0,
        a2: 0,
        a3: 0,
        a4: 0,
        a5: 0,
        a6: 0,
        a7: 0,
        s0: 0,
        s1: 0,
        s2: 0,
        s3: 0,
        s4: 0,
        s5: 0,
        s6: 0,
        s7: 0,
        s8: 0,
        s9: 0,
        s10: 0,
        s11: 0,
        gp: 0,
        tp: 0,
        sp: 0,
        pc: 0,
        mstatus: 0,
        mcause: 0,
        mtval: 0,
    },
    _running: false,
}; MAX_TASK];

pub fn task_create(task: extern "C" fn()) {
    unsafe {
        let i = allocate_task();

        CTX_TASKS[i].trap_frame.pc = task as usize;

        let task_stack_size = TASK_STACK_SIZE[i];

        // stack must be aligned by 16
        let task_stack_ptr =
            addr_of!(TASK_STACK) as usize + (task_stack_size * i) + task_stack_size - 4;
        let stack_ptr = task_stack_ptr - (task_stack_ptr % 0x10);
        CTX_TASKS[i].trap_frame.sp = stack_ptr;
    }
}

pub fn restore_task_context(id: usize, trap_frame: &mut TrapFrame) -> usize {
    unsafe {
        trap_frame.ra = CTX_TASKS[id].trap_frame.ra;
        trap_frame.sp = CTX_TASKS[id].trap_frame.sp;
        trap_frame.a0 = CTX_TASKS[id].trap_frame.a0;
        trap_frame.a1 = CTX_TASKS[id].trap_frame.a1;
        trap_frame.a2 = CTX_TASKS[id].trap_frame.a2;
        trap_frame.a3 = CTX_TASKS[id].trap_frame.a3;
        trap_frame.a4 = CTX_TASKS[id].trap_frame.a4;
        trap_frame.a5 = CTX_TASKS[id].trap_frame.a5;
        trap_frame.a6 = CTX_TASKS[id].trap_frame.a6;
        trap_frame.a7 = CTX_TASKS[id].trap_frame.a7;
        trap_frame.t0 = CTX_TASKS[id].trap_frame.t0;
        trap_frame.t1 = CTX_TASKS[id].trap_frame.t1;
        trap_frame.t2 = CTX_TASKS[id].trap_frame.t2;
        trap_frame.t3 = CTX_TASKS[id].trap_frame.t3;
        trap_frame.t4 = CTX_TASKS[id].trap_frame.t4;
        trap_frame.t5 = CTX_TASKS[id].trap_frame.t5;
        trap_frame.t6 = CTX_TASKS[id].trap_frame.t6;
        trap_frame.s0 = CTX_TASKS[id].trap_frame.s0;
        trap_frame.s1 = CTX_TASKS[id].trap_frame.s1;
        trap_frame.s2 = CTX_TASKS[id].trap_frame.s2;
        trap_frame.s3 = CTX_TASKS[id].trap_frame.s3;
        trap_frame.s4 = CTX_TASKS[id].trap_frame.s4;
        trap_frame.s5 = CTX_TASKS[id].trap_frame.s5;
        trap_frame.s6 = CTX_TASKS[id].trap_frame.s6;
        trap_frame.s7 = CTX_TASKS[id].trap_frame.s7;
        trap_frame.s8 = CTX_TASKS[id].trap_frame.s8;
        trap_frame.s9 = CTX_TASKS[id].trap_frame.s9;
        trap_frame.s10 = CTX_TASKS[id].trap_frame.s10;
        trap_frame.s11 = CTX_TASKS[id].trap_frame.s11;
        trap_frame.gp = CTX_TASKS[id].trap_frame.gp;
        trap_frame.tp = CTX_TASKS[id].trap_frame.tp;

        CTX_TASKS[id].trap_frame.pc
    }
}

pub fn save_task_context(id: usize, pc: usize, trap_frame: &TrapFrame) {
    unsafe {
        CTX_TASKS[id].trap_frame.ra = trap_frame.ra;
        CTX_TASKS[id].trap_frame.sp = trap_frame.sp;
        CTX_TASKS[id].trap_frame.a0 = trap_frame.a0;
        CTX_TASKS[id].trap_frame.a1 = trap_frame.a1;
        CTX_TASKS[id].trap_frame.a2 = trap_frame.a2;
        CTX_TASKS[id].trap_frame.a3 = trap_frame.a3;
        CTX_TASKS[id].trap_frame.a4 = trap_frame.a4;
        CTX_TASKS[id].trap_frame.a5 = trap_frame.a5;
        CTX_TASKS[id].trap_frame.a6 = trap_frame.a6;
        CTX_TASKS[id].trap_frame.a7 = trap_frame.a7;
        CTX_TASKS[id].trap_frame.t0 = trap_frame.t0;
        CTX_TASKS[id].trap_frame.t1 = trap_frame.t1;
        CTX_TASKS[id].trap_frame.t2 = trap_frame.t2;
        CTX_TASKS[id].trap_frame.t3 = trap_frame.t3;
        CTX_TASKS[id].trap_frame.t4 = trap_frame.t4;
        CTX_TASKS[id].trap_frame.t5 = trap_frame.t5;
        CTX_TASKS[id].trap_frame.t6 = trap_frame.t6;
        CTX_TASKS[id].trap_frame.s0 = trap_frame.s0;
        CTX_TASKS[id].trap_frame.s1 = trap_frame.s1;
        CTX_TASKS[id].trap_frame.s2 = trap_frame.s2;
        CTX_TASKS[id].trap_frame.s3 = trap_frame.s3;
        CTX_TASKS[id].trap_frame.s4 = trap_frame.s4;
        CTX_TASKS[id].trap_frame.s5 = trap_frame.s5;
        CTX_TASKS[id].trap_frame.s6 = trap_frame.s6;
        CTX_TASKS[id].trap_frame.s7 = trap_frame.s7;
        CTX_TASKS[id].trap_frame.s8 = trap_frame.s8;
        CTX_TASKS[id].trap_frame.s9 = trap_frame.s9;
        CTX_TASKS[id].trap_frame.s10 = trap_frame.s10;
        CTX_TASKS[id].trap_frame.s11 = trap_frame.s11;
        CTX_TASKS[id].trap_frame.gp = trap_frame.gp;
        CTX_TASKS[id].trap_frame.tp = trap_frame.tp;

        CTX_TASKS[id].trap_frame.pc = pc;
    }
}

pub fn task_switch(trap_frame: &mut TrapFrame) {
    let old_mepc = trap_frame.pc;

    save_task_context(current_task(), old_mepc, trap_frame);

    next_task();

    let new_pc = restore_task_context(current_task(), trap_frame);
    trap_frame.pc = new_pc;

    // debug aid! remove when not needed anymore!!!!!
    // static mut CNT: u32 = 0;
    // if CTX_NOW == 0 {
    //     if CNT < 5_000 {
    //         CNT += 1;
    //     } else {
    //         CNT = 0;
    //         info!("@@@ Task {} PC = {:x} {:?}", CTX_NOW, new_pc, trap_frame);
    //     }
    // }
}

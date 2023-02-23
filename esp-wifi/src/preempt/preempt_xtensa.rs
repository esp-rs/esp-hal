use super::*;
use crate::hal::trapframe::TrapFrame;

#[derive(Debug, Clone, Copy)]
pub struct TaskContext {
    trap_frame: TrapFrame,
}

static mut CTX_TASKS: [TaskContext; MAX_TASK] = [TaskContext {
    trap_frame: TrapFrame {
        PC: 0,
        PS: 0,
        A0: 0,
        A1: 0,
        A2: 0,
        A3: 0,
        A4: 0,
        A5: 0,
        A6: 0,
        A7: 0,
        A8: 0,
        A9: 0,
        A10: 0,
        A11: 0,
        A12: 0,
        A13: 0,
        A14: 0,
        A15: 0,
        SAR: 0,
        EXCCAUSE: 0,
        EXCVADDR: 0,
        LBEG: 0,
        LEND: 0,
        LCOUNT: 0,
        THREADPTR: 0,
        SCOMPARE1: 0,
        BR: 0,
        ACCLO: 0,
        ACCHI: 0,
        M0: 0,
        M1: 0,
        M2: 0,
        M3: 0,
        F64R_LO: 0,
        F64R_HI: 0,
        F64S: 0,
        FCR: 0,
        FSR: 0,
        F0: 0,
        F1: 0,
        F2: 0,
        F3: 0,
        F4: 0,
        F5: 0,
        F6: 0,
        F7: 0,
        F8: 0,
        F9: 0,
        F10: 0,
        F11: 0,
        F12: 0,
        F13: 0,
        F14: 0,
        F15: 0,
    },
}; MAX_TASK];

pub fn task_create(task: extern "C" fn()) -> usize {
    unsafe {
        let i = TASK_TOP;
        TASK_TOP += 1;

        CTX_TASKS[i].trap_frame.PC = task as u32;

        let task_stack_size = TASK_STACK_SIZE[i];

        // stack must be aligned by 16
        let task_stack_ptr = (&TASK_STACK as *const _ as usize
            + (task_stack_size as usize * i as usize)
            + task_stack_size as usize
            - 4) as u32;
        let stack_ptr = task_stack_ptr - (task_stack_ptr % 0x10);
        CTX_TASKS[i].trap_frame.A1 = stack_ptr;

        CTX_TASKS[i].trap_frame.PS = 0x00040000 | (1 & 3) << 16; // For windowed ABI set WOE and CALLINC (pretend task was 'call4'd).

        CTX_TASKS[i].trap_frame.A0 = 0;

        *((task_stack_ptr - 4) as *mut u32) = 0;
        *((task_stack_ptr - 8) as *mut u32) = 0;
        *((task_stack_ptr - 12) as *mut u32) = stack_ptr;
        *((task_stack_ptr - 16) as *mut u32) = 0;

        CTX_NOW = i;
        i
    }
}

pub fn task_to_trap_frame(id: usize, trap_frame: &mut TrapFrame) {
    unsafe {
        trap_frame.PC = CTX_TASKS[id].trap_frame.PC;
        trap_frame.PS = CTX_TASKS[id].trap_frame.PS;
        trap_frame.A0 = CTX_TASKS[id].trap_frame.A0;
        trap_frame.A1 = CTX_TASKS[id].trap_frame.A1;
        trap_frame.A2 = CTX_TASKS[id].trap_frame.A2;
        trap_frame.A3 = CTX_TASKS[id].trap_frame.A3;
        trap_frame.A4 = CTX_TASKS[id].trap_frame.A4;
        trap_frame.A5 = CTX_TASKS[id].trap_frame.A5;
        trap_frame.A6 = CTX_TASKS[id].trap_frame.A6;
        trap_frame.A7 = CTX_TASKS[id].trap_frame.A7;
        trap_frame.A8 = CTX_TASKS[id].trap_frame.A8;
        trap_frame.A9 = CTX_TASKS[id].trap_frame.A9;
        trap_frame.A10 = CTX_TASKS[id].trap_frame.A10;
        trap_frame.A11 = CTX_TASKS[id].trap_frame.A11;
        trap_frame.A12 = CTX_TASKS[id].trap_frame.A12;
        trap_frame.A13 = CTX_TASKS[id].trap_frame.A13;
        trap_frame.A14 = CTX_TASKS[id].trap_frame.A14;
        trap_frame.A15 = CTX_TASKS[id].trap_frame.A15;
        trap_frame.SAR = CTX_TASKS[id].trap_frame.SAR;
        trap_frame.EXCCAUSE = CTX_TASKS[id].trap_frame.EXCCAUSE;
        trap_frame.EXCVADDR = CTX_TASKS[id].trap_frame.EXCVADDR;
        trap_frame.LBEG = CTX_TASKS[id].trap_frame.LBEG;
        trap_frame.LEND = CTX_TASKS[id].trap_frame.LEND;
        trap_frame.LCOUNT = CTX_TASKS[id].trap_frame.LCOUNT;
        trap_frame.THREADPTR = CTX_TASKS[id].trap_frame.THREADPTR;
        trap_frame.SCOMPARE1 = CTX_TASKS[id].trap_frame.SCOMPARE1;
        trap_frame.BR = CTX_TASKS[id].trap_frame.BR;
        trap_frame.ACCLO = CTX_TASKS[id].trap_frame.ACCLO;
        trap_frame.ACCHI = CTX_TASKS[id].trap_frame.ACCHI;
        trap_frame.M0 = CTX_TASKS[id].trap_frame.M0;
        trap_frame.M1 = CTX_TASKS[id].trap_frame.M1;
        trap_frame.M2 = CTX_TASKS[id].trap_frame.M2;
        trap_frame.M3 = CTX_TASKS[id].trap_frame.M3;
        trap_frame.F64R_LO = CTX_TASKS[id].trap_frame.F64R_LO;
        trap_frame.F64R_HI = CTX_TASKS[id].trap_frame.F64R_HI;
        trap_frame.F64S = CTX_TASKS[id].trap_frame.F64S;
        trap_frame.FCR = CTX_TASKS[id].trap_frame.FCR;
        trap_frame.FSR = CTX_TASKS[id].trap_frame.FSR;
        trap_frame.F0 = CTX_TASKS[id].trap_frame.F0;
        trap_frame.F1 = CTX_TASKS[id].trap_frame.F1;
        trap_frame.F2 = CTX_TASKS[id].trap_frame.F2;
        trap_frame.F3 = CTX_TASKS[id].trap_frame.F3;
        trap_frame.F4 = CTX_TASKS[id].trap_frame.F4;
        trap_frame.F5 = CTX_TASKS[id].trap_frame.F5;
        trap_frame.F6 = CTX_TASKS[id].trap_frame.F6;
        trap_frame.F7 = CTX_TASKS[id].trap_frame.F7;
        trap_frame.F8 = CTX_TASKS[id].trap_frame.F8;
        trap_frame.F9 = CTX_TASKS[id].trap_frame.F9;
        trap_frame.F10 = CTX_TASKS[id].trap_frame.F10;
        trap_frame.F11 = CTX_TASKS[id].trap_frame.F11;
        trap_frame.F12 = CTX_TASKS[id].trap_frame.F12;
        trap_frame.F13 = CTX_TASKS[id].trap_frame.F13;
        trap_frame.F14 = CTX_TASKS[id].trap_frame.F14;
        trap_frame.F15 = CTX_TASKS[id].trap_frame.F15;
    }
}

pub fn trap_frame_to_task(id: usize, trap_frame: &TrapFrame) {
    unsafe {
        CTX_TASKS[id].trap_frame.PC = trap_frame.PC;
        CTX_TASKS[id].trap_frame.PS = trap_frame.PS;
        CTX_TASKS[id].trap_frame.A0 = trap_frame.A0;
        CTX_TASKS[id].trap_frame.A1 = trap_frame.A1;
        CTX_TASKS[id].trap_frame.A2 = trap_frame.A2;
        CTX_TASKS[id].trap_frame.A3 = trap_frame.A3;
        CTX_TASKS[id].trap_frame.A4 = trap_frame.A4;
        CTX_TASKS[id].trap_frame.A5 = trap_frame.A5;
        CTX_TASKS[id].trap_frame.A6 = trap_frame.A6;
        CTX_TASKS[id].trap_frame.A7 = trap_frame.A7;
        CTX_TASKS[id].trap_frame.A8 = trap_frame.A8;
        CTX_TASKS[id].trap_frame.A9 = trap_frame.A9;
        CTX_TASKS[id].trap_frame.A10 = trap_frame.A10;
        CTX_TASKS[id].trap_frame.A11 = trap_frame.A11;
        CTX_TASKS[id].trap_frame.A12 = trap_frame.A12;
        CTX_TASKS[id].trap_frame.A13 = trap_frame.A13;
        CTX_TASKS[id].trap_frame.A14 = trap_frame.A14;
        CTX_TASKS[id].trap_frame.A15 = trap_frame.A15;
        CTX_TASKS[id].trap_frame.SAR = trap_frame.SAR;
        CTX_TASKS[id].trap_frame.EXCCAUSE = trap_frame.EXCCAUSE;
        CTX_TASKS[id].trap_frame.EXCVADDR = trap_frame.EXCVADDR;
        CTX_TASKS[id].trap_frame.LBEG = trap_frame.LBEG;
        CTX_TASKS[id].trap_frame.LEND = trap_frame.LEND;
        CTX_TASKS[id].trap_frame.LCOUNT = trap_frame.LCOUNT;
        CTX_TASKS[id].trap_frame.THREADPTR = trap_frame.THREADPTR;
        CTX_TASKS[id].trap_frame.SCOMPARE1 = trap_frame.SCOMPARE1;
        CTX_TASKS[id].trap_frame.BR = trap_frame.BR;
        CTX_TASKS[id].trap_frame.ACCLO = trap_frame.ACCLO;
        CTX_TASKS[id].trap_frame.ACCHI = trap_frame.ACCHI;
        CTX_TASKS[id].trap_frame.M0 = trap_frame.M0;
        CTX_TASKS[id].trap_frame.M1 = trap_frame.M1;
        CTX_TASKS[id].trap_frame.M2 = trap_frame.M2;
        CTX_TASKS[id].trap_frame.M3 = trap_frame.M3;
        CTX_TASKS[id].trap_frame.F64R_LO = trap_frame.F64R_LO;
        CTX_TASKS[id].trap_frame.F64R_HI = trap_frame.F64R_HI;
        CTX_TASKS[id].trap_frame.F64S = trap_frame.F64S;
        CTX_TASKS[id].trap_frame.FCR = trap_frame.FCR;
        CTX_TASKS[id].trap_frame.FSR = trap_frame.FSR;
        CTX_TASKS[id].trap_frame.F0 = trap_frame.F0;
        CTX_TASKS[id].trap_frame.F1 = trap_frame.F1;
        CTX_TASKS[id].trap_frame.F2 = trap_frame.F2;
        CTX_TASKS[id].trap_frame.F3 = trap_frame.F3;
        CTX_TASKS[id].trap_frame.F4 = trap_frame.F4;
        CTX_TASKS[id].trap_frame.F5 = trap_frame.F5;
        CTX_TASKS[id].trap_frame.F6 = trap_frame.F6;
        CTX_TASKS[id].trap_frame.F7 = trap_frame.F7;
        CTX_TASKS[id].trap_frame.F8 = trap_frame.F8;
        CTX_TASKS[id].trap_frame.F9 = trap_frame.F9;
        CTX_TASKS[id].trap_frame.F10 = trap_frame.F10;
        CTX_TASKS[id].trap_frame.F11 = trap_frame.F11;
        CTX_TASKS[id].trap_frame.F12 = trap_frame.F12;
        CTX_TASKS[id].trap_frame.F13 = trap_frame.F13;
        CTX_TASKS[id].trap_frame.F14 = trap_frame.F14;
        CTX_TASKS[id].trap_frame.F15 = trap_frame.F15;
    }
}

pub fn next_task() {
    unsafe {
        CTX_NOW = (CTX_NOW + 1) % TASK_TOP;
    }
}

pub fn task_switch(trap_frame: &mut TrapFrame) {
    unsafe {
        if FIRST_SWITCH.load(Ordering::Relaxed) {
            FIRST_SWITCH.store(false, Ordering::Relaxed);
            TASK_TOP += 1;
            CTX_NOW = TASK_TOP - 1;
        }

        trap_frame_to_task(CTX_NOW, trap_frame);
        next_task();
        task_to_trap_frame(CTX_NOW, trap_frame);

        // debug aid! remove when not needed anymore!!!!!
        // static mut CNT: u32 = 0;
        // if CTX_NOW == 0 {
        //     if CNT < 2_000 {
        //         CNT += 1;
        //     } else {
        //         CNT = 0;
        //         log::info!("@@@ Task {} {:x?} ", 1, CTX_TASKS[1].trap_frame.PC);
        //     }
        // }
    };
}

pub fn current_task() -> usize {
    unsafe { CTX_NOW }
}

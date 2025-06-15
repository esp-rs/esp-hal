use esp_wifi_sys::c_types;

use super::*;
pub use crate::hal::interrupt::TrapFrame;

pub(crate) fn new_task_context(
    task: extern "C" fn(*mut c_types::c_void),
    param: *mut c_types::c_void,
    stack_top: *mut (),
) -> TrapFrame {
    let stack_top = stack_top as usize;
    let stack_top = stack_top - (stack_top % 16);

    TrapFrame {
        pc: task as usize,
        a0: param as usize,
        sp: stack_top,
        ..Default::default()
    }
}

pub(crate) fn restore_task_context(ctx: &mut Context, trap_frame: &mut TrapFrame) {
    trap_frame.ra = ctx.trap_frame.ra;
    trap_frame.sp = ctx.trap_frame.sp;
    trap_frame.a0 = ctx.trap_frame.a0;
    trap_frame.a1 = ctx.trap_frame.a1;
    trap_frame.a2 = ctx.trap_frame.a2;
    trap_frame.a3 = ctx.trap_frame.a3;
    trap_frame.a4 = ctx.trap_frame.a4;
    trap_frame.a5 = ctx.trap_frame.a5;
    trap_frame.a6 = ctx.trap_frame.a6;
    trap_frame.a7 = ctx.trap_frame.a7;
    trap_frame.t0 = ctx.trap_frame.t0;
    trap_frame.t1 = ctx.trap_frame.t1;
    trap_frame.t2 = ctx.trap_frame.t2;
    trap_frame.t3 = ctx.trap_frame.t3;
    trap_frame.t4 = ctx.trap_frame.t4;
    trap_frame.t5 = ctx.trap_frame.t5;
    trap_frame.t6 = ctx.trap_frame.t6;
    trap_frame.s0 = ctx.trap_frame.s0;
    trap_frame.s1 = ctx.trap_frame.s1;
    trap_frame.s2 = ctx.trap_frame.s2;
    trap_frame.s3 = ctx.trap_frame.s3;
    trap_frame.s4 = ctx.trap_frame.s4;
    trap_frame.s5 = ctx.trap_frame.s5;
    trap_frame.s6 = ctx.trap_frame.s6;
    trap_frame.s7 = ctx.trap_frame.s7;
    trap_frame.s8 = ctx.trap_frame.s8;
    trap_frame.s9 = ctx.trap_frame.s9;
    trap_frame.s10 = ctx.trap_frame.s10;
    trap_frame.s11 = ctx.trap_frame.s11;
    trap_frame.gp = ctx.trap_frame.gp;
    trap_frame.tp = ctx.trap_frame.tp;
    trap_frame.pc = ctx.trap_frame.pc;
}

pub(crate) fn save_task_context(ctx: &mut Context, trap_frame: &TrapFrame) {
    ctx.trap_frame.ra = trap_frame.ra;
    ctx.trap_frame.sp = trap_frame.sp;
    ctx.trap_frame.a0 = trap_frame.a0;
    ctx.trap_frame.a1 = trap_frame.a1;
    ctx.trap_frame.a2 = trap_frame.a2;
    ctx.trap_frame.a3 = trap_frame.a3;
    ctx.trap_frame.a4 = trap_frame.a4;
    ctx.trap_frame.a5 = trap_frame.a5;
    ctx.trap_frame.a6 = trap_frame.a6;
    ctx.trap_frame.a7 = trap_frame.a7;
    ctx.trap_frame.t0 = trap_frame.t0;
    ctx.trap_frame.t1 = trap_frame.t1;
    ctx.trap_frame.t2 = trap_frame.t2;
    ctx.trap_frame.t3 = trap_frame.t3;
    ctx.trap_frame.t4 = trap_frame.t4;
    ctx.trap_frame.t5 = trap_frame.t5;
    ctx.trap_frame.t6 = trap_frame.t6;
    ctx.trap_frame.s0 = trap_frame.s0;
    ctx.trap_frame.s1 = trap_frame.s1;
    ctx.trap_frame.s2 = trap_frame.s2;
    ctx.trap_frame.s3 = trap_frame.s3;
    ctx.trap_frame.s4 = trap_frame.s4;
    ctx.trap_frame.s5 = trap_frame.s5;
    ctx.trap_frame.s6 = trap_frame.s6;
    ctx.trap_frame.s7 = trap_frame.s7;
    ctx.trap_frame.s8 = trap_frame.s8;
    ctx.trap_frame.s9 = trap_frame.s9;
    ctx.trap_frame.s10 = trap_frame.s10;
    ctx.trap_frame.s11 = trap_frame.s11;
    ctx.trap_frame.gp = trap_frame.gp;
    ctx.trap_frame.tp = trap_frame.tp;
    ctx.trap_frame.pc = trap_frame.pc;
}

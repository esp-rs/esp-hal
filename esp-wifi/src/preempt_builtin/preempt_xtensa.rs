use esp_wifi_sys::c_types;

use super::*;
pub use crate::hal::trapframe::TrapFrame;

pub(crate) fn new_task_context(
    task_fn: extern "C" fn(*mut c_types::c_void),
    param: *mut c_types::c_void,
    stack_top: *mut (),
) -> TrapFrame {
    // stack must be aligned by 16
    let stack_top = stack_top as u32;
    let stack_top = stack_top - (stack_top % 16);

    unsafe {
        *((stack_top - 4) as *mut u32) = 0;
        *((stack_top - 8) as *mut u32) = 0;
        *((stack_top - 12) as *mut u32) = stack_top;
        *((stack_top - 16) as *mut u32) = 0;
    }

    TrapFrame {
        PC: task_fn as usize as u32,
        A0: 0,
        A1: stack_top,
        A6: param as usize as u32,

        // For windowed ABI set WOE and CALLINC (pretend task was 'call4'd)
        PS: 0x00040000 | ((1 & 3) << 16),

        ..Default::default()
    }
}

pub(crate) fn restore_task_context(ctx: &mut Context, trap_frame: &mut TrapFrame) {
    *trap_frame = ctx.trap_frame;
}

pub(crate) fn save_task_context(ctx: &mut Context, trap_frame: &TrapFrame) {
    ctx.trap_frame = *trap_frame;
}

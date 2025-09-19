use core::ffi::c_void;

pub(crate) use esp_hal::trapframe::TrapFrame as CpuContext;
use esp_hal::{xtensa_lx, xtensa_lx_rt};

use crate::SCHEDULER;

extern "C" fn idle_hook() -> ! {
    loop {
        unsafe { core::arch::asm!("waiti 0") };
    }
}

#[unsafe(naked)]
extern "C" fn idle_entry() -> ! {
    core::arch::naked_asm!("call4 {idle_hook}", idle_hook = sym idle_hook);
}

pub(crate) fn set_idle_hook_entry(idle_context: &mut CpuContext) {
    // Point idle context PC at the assembly that calls the idle hook. We need a new stack
    // frame for the idle task on the main stack.
    idle_context.PC = idle_entry as usize as u32;
    // Set a valid processor status value
    let current_ps;
    unsafe { core::arch::asm!("rsr.ps {0}", out(reg) current_ps, options(nostack)) };
    idle_context.PS = current_ps;
}

pub(crate) fn new_task_context(
    task_fn: extern "C" fn(*mut c_void),
    param: *mut c_void,
    stack_top: *mut (),
) -> CpuContext {
    // stack must be aligned by 16
    let stack_top = stack_top as u32;
    let stack_top = stack_top - (stack_top % 16);

    unsafe {
        *((stack_top - 4) as *mut u32) = 0;
        *((stack_top - 8) as *mut u32) = 0;
        *((stack_top - 12) as *mut u32) = stack_top;
        *((stack_top - 16) as *mut u32) = 0;
    }

    CpuContext {
        PC: task_fn as usize as u32,
        A0: 0,
        A1: stack_top,
        A6: param as usize as u32,

        // For windowed ABI set WOE and CALLINC (pretend task was 'call4'd)
        PS: 0x00040000 | ((1 & 3) << 16),

        ..Default::default()
    }
}

pub(crate) fn task_switch(
    current_context: *mut CpuContext,
    next_context: *mut CpuContext,
    trap_frame: &mut CpuContext,
) {
    if !current_context.is_null() {
        unsafe { core::ptr::copy_nonoverlapping(trap_frame, current_context, 1) };
    }
    unsafe { core::ptr::copy_nonoverlapping(next_context, trap_frame, 1) };
}

// ESP32 uses Software1 (priority 3) for task switching, because it reserves
// Software0 for the Bluetooth stack.
const SW_INTERRUPT: u32 = if cfg!(esp32) { 1 << 29 } else { 1 << 7 };

pub(crate) fn setup_multitasking() {
    unsafe {
        xtensa_lx::interrupt::enable_mask(
            SW_INTERRUPT
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask(),
        );
    }
}

#[allow(non_snake_case)]
#[esp_hal::ram]
#[cfg_attr(not(esp32), unsafe(export_name = "Software0"))]
#[cfg_attr(esp32, unsafe(export_name = "Software1"))]
fn task_switch_interrupt(context: &mut CpuContext) {
    unsafe { xtensa_lx_rt::xtensa_lx::interrupt::clear(SW_INTERRUPT) };

    SCHEDULER.with(|scheduler| scheduler.switch_task(context));
}

#[inline]
pub(crate) fn yield_task() {
    unsafe { xtensa_lx::interrupt::set(SW_INTERRUPT) };
}

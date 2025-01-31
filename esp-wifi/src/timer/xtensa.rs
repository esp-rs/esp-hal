use esp_hal::interrupt::InterruptHandler;

use crate::{
    hal::{interrupt, time::Rate, trapframe::TrapFrame, xtensa_lx, xtensa_lx_rt},
    preempt::task_switch,
    TimeBase,
};

/// The timer responsible for time slicing.
const TIMESLICE_FREQUENCY: Rate = Rate::from_hz(crate::CONFIG.tick_rate_hz);

use super::TIMER;

// Time keeping
pub const TICKS_PER_SECOND: u64 = 1_000_000;

/// This function must not be called in a critical section. Doing so may return
/// an incorrect value.
pub(crate) fn systimer_count() -> u64 {
    esp_hal::time::now().duration_since_epoch().as_micros()
}

pub(crate) fn setup_timer(mut timer1: TimeBase) {
    timer1.set_interrupt_handler(InterruptHandler::new(
        unsafe { core::mem::transmute::<*const (), extern "C" fn()>(handler as *const ()) },
        interrupt::Priority::Priority2,
    ));
    unwrap!(timer1.start(TIMESLICE_FREQUENCY.as_duration()));
    TIMER.with(|timer| {
        timer1.enable_interrupt(true);
        timer.replace(timer1);
    });
}

pub(crate) fn disable_timer() {
    TIMER.with(|timer| {
        let timer = unwrap!(timer.as_mut());
        timer.enable_interrupt(false);
        unwrap!(timer.cancel());
    });
}

pub(crate) fn setup_multitasking() {
    unsafe {
        let enabled = xtensa_lx::interrupt::disable();
        xtensa_lx::interrupt::enable_mask(
            1 << 29 // Software1
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level2.mask()
                | xtensa_lx_rt::interrupt::CpuInterruptLevel::Level6.mask() | enabled,
        );
    }
}

pub(crate) fn disable_multitasking() {
    xtensa_lx::interrupt::disable_mask(
        1 << 29, // Disable Software1
    );
}

fn do_task_switch(context: &mut TrapFrame) {
    TIMER.with(|timer| {
        let timer = unwrap!(timer.as_mut());
        timer.clear_interrupt();
    });

    task_switch(context);
}

extern "C" fn handler(context: &mut TrapFrame) {
    do_task_switch(context);
}

#[allow(non_snake_case)]
#[no_mangle]
fn Software1(_level: u32, context: &mut TrapFrame) {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.intclear  {0}", in(reg) intr, options(nostack));
    }

    do_task_switch(context);
}

pub(crate) fn yield_task() {
    let intr = 1 << 29;
    unsafe {
        core::arch::asm!("wsr.intset  {0}", in(reg) intr, options(nostack));
    }
}

// TODO: use an Instance type instead...
pub(crate) fn time_diff(start: u64, end: u64) -> u64 {
    end.wrapping_sub(start)
}

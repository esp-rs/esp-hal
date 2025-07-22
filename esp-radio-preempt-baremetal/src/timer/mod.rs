#[cfg_attr(target_arch = "riscv32", path = "riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "xtensa.rs")]
mod arch_specific;

pub(crate) use arch_specific::*;
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    sync::Locked,
    trapframe::TrapFrame,
};

use crate::TimeBase;

/// The timer responsible for time slicing.
pub(crate) static TIMER: Locked<Option<TimeBase>> = Locked::new(None);

pub(crate) fn initialized() -> bool {
    TIMER.with(|timer| timer.is_some())
}

pub(crate) fn setup_timebase(mut timer: TimeBase) {
    // The timer needs to tick at Priority 1 to prevent accidentally interrupting
    // priority 1 limited locks.
    let cb: extern "C" fn() = unsafe { core::mem::transmute(timer_tick_handler as *const ()) };
    let handler = InterruptHandler::new(cb, Priority::Priority1);

    timer.set_interrupt_handler(handler);
    TIMER.with(|t| {
        timer.enable_interrupt(true);
        t.replace(timer);
    });
}

pub(crate) fn clear_timer_interrupt() {
    TIMER.with(|timer| {
        unwrap!(timer.as_mut()).clear_interrupt();
    });
}

pub(crate) fn disable_timebase() {
    TIMER.with(|timer| {
        let mut timer = unwrap!(timer.take());
        timer.enable_interrupt(false);
        unwrap!(timer.cancel());
    });
}

#[esp_hal::ram]
extern "C" fn timer_tick_handler(context: &mut TrapFrame) {
    clear_timer_interrupt();

    // `task_switch` must be called on a single interrupt priority level only.
    // Because on ESP32 the software interrupt is triggered at priority 3 but
    // the timer interrupt is triggered at priority 1, we need to trigger the
    // software interrupt manually.
    if cfg!(esp32) {
        yield_task();
    } else {
        crate::task::task_switch(context);
    }
}

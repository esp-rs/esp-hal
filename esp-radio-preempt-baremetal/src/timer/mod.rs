#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
mod arch_specific;

pub(crate) use arch_specific::*;
use esp_hal::{
    interrupt::{InterruptHandler, Priority},
    sync::Locked,
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

    // Register the interrupt handler without nesting to satisfy the requirements of the task
    // switching code
    #[cfg(riscv)]
    let handler = InterruptHandler::new_not_nested(cb, Priority::Priority1);

    #[cfg(xtensa)]
    let handler = InterruptHandler::new(cb, Priority::Priority1);

    timer.set_interrupt_handler(handler);
    TIMER.with(|t| {
        timer.listen();
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
        timer.unlisten();
        unwrap!(timer.cancel());
    });
}

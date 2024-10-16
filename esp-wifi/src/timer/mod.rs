use core::cell::RefCell;

use critical_section::Mutex;

#[cfg_attr(esp32, path = "timer_esp32.rs")]
#[cfg_attr(esp32c2, path = "timer_esp32c2.rs")]
#[cfg_attr(esp32c3, path = "timer_esp32c3.rs")]
#[cfg_attr(esp32c6, path = "timer_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "timer_esp32h2.rs")]
#[cfg_attr(esp32s3, path = "timer_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "timer_esp32s2.rs")]
mod chip_specific;

#[cfg_attr(any(esp32, esp32s2, esp32s3), path = "xtensa.rs")]
#[cfg_attr(any(esp32c2, esp32c3, esp32c6, esp32h2), path = "riscv.rs")]
mod arch_specific;

pub(crate) use arch_specific::*;
pub(crate) use chip_specific::*;

use crate::TimeBase;

pub(crate) static TIMER: Mutex<RefCell<Option<TimeBase>>> = Mutex::new(RefCell::new(None));

pub(crate) fn setup_timer_isr(timebase: TimeBase) -> Result<(), esp_hal::timer::Error> {
    setup_radio_isr();

    setup_timer(timebase)?;

    setup_multitasking();

    yield_task();
    Ok(())
}

pub(crate) fn shutdown_timer_isr() -> Result<(), esp_hal::timer::Error> {
    shutdown_radio_isr();

    disable_timer()?;

    disable_multitasking();
    Ok(())
}

#[allow(unused)]
pub(crate) fn micros_to_ticks(us: u64) -> u64 {
    us * (TICKS_PER_SECOND / 1_000_000)
}

#[allow(unused)]
pub(crate) fn millis_to_ticks(ms: u64) -> u64 {
    ms * (TICKS_PER_SECOND / 1_000)
}

#[allow(unused)]
pub(crate) fn ticks_to_micros(ticks: u64) -> u64 {
    ticks / (TICKS_PER_SECOND / 1_000_000)
}

#[allow(unused)]
pub(crate) fn ticks_to_millis(ticks: u64) -> u64 {
    ticks / (TICKS_PER_SECOND / 1_000)
}

/// Do not call this in a critical section!
pub(crate) fn elapsed_time_since(start: u64) -> u64 {
    let now = get_systimer_count();
    time_diff(start, now)
}

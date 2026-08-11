//! The low-power path of esp32c2 and esp32c3.
//!
//! These chips have no `ext1`. Their per-pin path gives each low-power pad its own level, keeps no
//! power domain powered, and reaches the pad through the digital IO MUX. There is thus nothing to
//! allocate, and no pad to move to another IO MUX.

use super::{Armed, LP_NUMBERS, NO_LP_NUMBER};
use crate::{
    gpio::{Level, lp_io::low_level},
    peripherals::LPWR,
    rtc_cntl::{WakeupReason, WakeupSource, sleep::WrappedSleepConfig},
};

/// These chips have no separate path to clear, because all their pads use the GPIO wakeup source.
pub(super) fn disable() {}

pub(super) fn allocate(armed: &[Armed], config: &mut WrappedSleepConfig<'_>) {
    // A pad outside this sleep must not wake the chip.
    for &lp in LP_NUMBERS.iter().filter(|&&lp| lp != NO_LP_NUMBER) {
        low_level::apply_wakeup(lp, false, Level::Low);
    }

    for pin in armed {
        // Deep sleep powers the digital IO MUX down, and the pull resistors are part of it. The pad
        // must therefore keep its own state through the sleep, or it floats to an unknown level.
        // The hold outlives the sleep: neither the wake of a light sleep nor the reset of a
        // deep-sleep wake releases it, so it is released in software.
        if config.is_deep_sleep() {
            low_level::pad_hold(pin.lp, Some(true));
            super::hold_taken(pin.gpio);
        }

        low_level::apply_wakeup(pin.lp, true, pin.level);
    }

    prepare_gpio_wakeup();
}

/// Returns whether this pad ended the last sleep through the per-pin path.
pub(super) fn caused_wakeup(gpio: u8, cause: WakeupReason) -> bool {
    // Only a low-power pad can take this path.
    let Some(lp) = super::lp_number(gpio) else {
        return false;
    };

    let status = cfg_select! {
        esp32c2 => LPWR::regs()
            .cntl_gpio_wakeup()
            .read()
            .gpio_wakeup_status()
            .bits(),
        esp32c3 => LPWR::regs()
            .gpio_wakeup()
            .read()
            .gpio_wakeup_status()
            .bits(),
    };

    cause.contains(WakeupSource::Gpio) && status & (1 << lp) != 0
}

/// Releases the pads that the previous run armed for a deep sleep.
///
/// The per-pin registers keep their wakeup enables through the wake, so they still name the pads
/// that the previous run armed.
pub(super) fn wake_io_reset() {
    let armed = low_level::wakeup_enabled_mask();

    for lp in super::low_power_numbers() {
        if armed & (1 << lp) != 0 {
            low_level::pad_hold(lp, Some(false));
        }
    }
}

/// Enables the clock of the pad-scanning logic that this path needs, and clears the status of the
/// previous wake.
fn prepare_gpio_wakeup() {
    let gpio_wakeup = cfg_select! {
        esp32c2 => LPWR::regs().cntl_gpio_wakeup(),
        esp32c3 => LPWR::regs().gpio_wakeup(),
    };

    gpio_wakeup.modify(|_, w| w.gpio_pin_clk_gate().set_bit());
    LPWR::regs()
        .ext_wakeup_conf()
        .modify(|_, w| w.gpio_wakeup_filter().set_bit());

    gpio_wakeup.modify(|_, w| w.gpio_wakeup_status_clr().set_bit());
    gpio_wakeup.modify(|_, w| w.gpio_wakeup_status_clr().clear_bit());
}

//! The low-power path of esp32c2 and esp32c3.
//!
//! These chips have no `ext1`. Their per-pin path gives every low-power pad a level of its own, it
//! costs no power domain, and it reaches the pad through the digital IO MUX, so there is nothing to
//! allocate and no pad to hand over.

use super::{Armed, low_power_pads};
use crate::{
    gpio::{Level, lp_io::low_level},
    peripherals::LPWR,
    rtc_cntl::sleep::{SleepKind, WrappedSleepConfig},
};

/// These chips have no path of their own to clear: the pads share the GPIO wakeup source.
pub(super) fn disable() {}

pub(super) fn allocate(armed: &[Armed], kind: SleepKind, _config: &mut WrappedSleepConfig<'_>) {
    // The pads this sleep did not choose must not wake it.
    for (_, lp) in low_power_pads() {
        low_level::apply_wakeup(lp, false, Level::Low);
    }

    for pin in armed {
        // ESP-IDF drives the pad to its wake level with the pull resistors, so that a deep sleep,
        // which isolates the pad, cannot lose it.
        if kind == SleepKind::Deep {
            low_level::pullup_enable(pin.lp, pin.level == Level::Low);
            low_level::pulldown_enable(pin.lp, pin.level == Level::High);
        }

        low_level::apply_wakeup(pin.lp, true, pin.level);
    }

    prepare_gpio_wakeup();
}

/// Clocks the pad-scanning logic that this path needs, and clears the status a previous wake left
/// latched.
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

//! The low-power path of esp32c2 and esp32c3.
//!
//! These chips have no `ext1`. Their per-pin path gives every low-power pad a level of its own, it
//! costs no power domain, and it reaches the pad through the digital IO MUX, so there is nothing to
//! allocate and no pad to hand over.

use super::{Armed, LP_NUMBERS, NO_LP_NUMBER};
use crate::{
    gpio::{Level, lp_io::low_level},
    peripherals::LPWR,
    rtc_cntl::sleep::{SleepKind, WrappedSleepConfig},
};

/// These chips have no path of their own to clear: the pads share the GPIO wakeup source.
pub(super) fn disable() {}

pub(super) fn allocate(armed: &[Armed], _kind: SleepKind, _config: &mut WrappedSleepConfig<'_>) {
    // The pads this sleep did not choose must not wake it.
    for &lp in LP_NUMBERS.iter().filter(|&&lp| lp != NO_LP_NUMBER) {
        low_level::apply_wakeup(lp, false, Level::Low);
    }

    for pin in armed {
        // The pull resistors are the pin's own configuration, which these chips keep in force
        // because their low-power pull registers are the digital ones.
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

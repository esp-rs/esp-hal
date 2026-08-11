//! The low-power path of esp32p4 and esp32s31.
//!
//! On these chips, `ext1` has one level per pad, covers every low-power pad, and does not increase
//! the sleep current. There is thus nothing to compare, and every pin goes to `ext1`. These chips
//! also have a per-pin path, which stays unused until esp-hal supports edge triggers.
//!
//! This `ext1` selects the pads by pin number, and not by low-power number like the generation
//! before it.

use super::{Armed, prepare_pad};
use crate::{
    gpio::{Level, lp_io::low_level},
    peripherals::PMU,
    rtc_cntl::{WakeupReason, WakeupSource, sleep::WrappedSleepConfig},
};

/// Clears the paths that the pins of this chip can take, so that no bit of a previous sleep wakes
/// the chip.
pub(super) fn disable() {
    WakeupSource::Ext1.disable();
}

pub(super) fn allocate(armed: &[Armed], config: &mut WrappedSleepConfig<'_>) {
    if armed.is_empty() {
        write_ext1(0, 0);
        WakeupSource::Ext1.disable();
        return;
    }

    let mut pads = 0;
    let mut levels = 0;
    for pin in armed {
        let bit = 1 << pin.gpio;

        pads |= bit;
        if pin.level == Level::High {
            levels |= bit;
        }

        prepare_pad(pin, config.is_deep_sleep());
    }

    write_ext1(pads, levels);
    WakeupSource::Ext1.enable();
}

/// Returns whether this pad ended the last sleep through `ext1`, which is the only low-power path
/// that these chips use.
pub(super) fn caused_wakeup(gpio: u8, cause: WakeupReason) -> bool {
    // Only a low-power pad can take this path. This `ext1` selects the pads by pin number, and the
    // status register covers the pin number of every low-power pad of these chips.
    if super::lp_number(gpio).is_none() {
        return false;
    }

    cause.contains(WakeupSource::Ext1)
        && PMU::regs()
            .ext_wakeup_st()
            .read()
            .ext_wakeup_status()
            .bits()
            & (1 << gpio)
            != 0
}

/// Releases the pads that the previous run armed for a deep sleep.
///
/// This `ext1` selects the pads by pin number, and it keeps the selection through the wake. The
/// hold register uses the low-power number, so the release needs both numbers.
pub(super) fn wake_io_reset() {
    let armed = PMU::regs().ext_wakeup_sel().read().ext_wakeup_sel().bits();

    for (gpio, lp) in super::low_power_pads() {
        if armed & (1 << gpio) != 0 {
            low_level::pad_hold(lp, false);
        }
    }

    // The pads are free now, so clear the selection that armed them.
    PMU::regs().ext_wakeup_sel().reset();
}

fn write_ext1(pads: u32, levels: u32) {
    PMU::regs()
        .ext_wakeup_cntl()
        .modify(|_, w| w.ext_wakeup_status_clr().set_bit());
    PMU::regs()
        .ext_wakeup_cntl()
        .modify(|_, w| w.ext_wakeup_status_clr().clear_bit());
    PMU::regs()
        .ext_wakeup_sel()
        .write(|w| unsafe { w.ext_wakeup_sel().bits(pads) });
    PMU::regs()
        .ext_wakeup_lv()
        .write(|w| unsafe { w.ext_wakeup_lv().bits(levels) });
}

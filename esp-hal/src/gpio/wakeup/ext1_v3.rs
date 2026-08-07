//! The low-power path of esp32p4 and esp32s31.
//!
//! `ext1` has a level per pad here, and it covers every low-power pad at no cost in sleep current,
//! so there is nothing to weigh: every pin goes there. The per-pin path these chips also have
//! stays unused until edge support arrives.
//!
//! Unlike the generation before it, this `ext1` selects the pads by pin number.

use super::{Armed, prepare_pad};
use crate::{
    gpio::Level,
    peripherals::PMU,
    rtc_cntl::{
        WakeupReason,
        WakeupSource,
        sleep::{SleepKind, WrappedSleepConfig},
    },
};

/// Clears the paths that this chip's pins can take, so that a stale bit cannot wake the chip.
pub(super) fn disable() {
    WakeupSource::Ext1.disable();
}

pub(super) fn allocate(armed: &[Armed], kind: SleepKind, _config: &mut WrappedSleepConfig<'_>) {
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

        prepare_pad(pin, kind);
    }

    write_ext1(pads, levels);
    WakeupSource::Ext1.enable();
}

/// Reports whether this pad ended the last sleep through `ext1`, the one low-power path these
/// chips use.
pub(super) fn caused_wakeup(gpio: u8, cause: WakeupReason) -> bool {
    // Only a low-power pad can take this path. This `ext1` selects the pads by pin number, and
    // every low-power pad here has a pin number the status register covers.
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

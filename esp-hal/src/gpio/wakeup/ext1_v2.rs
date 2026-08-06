//! The low-power path of esp32c5, esp32c6, esp32c61 and esp32h2.
//!
//! `ext1` has a level per pad here, and it covers every low-power pad at no cost in sleep current,
//! so there is nothing to weigh: every pin goes there. The per-pin path these chips also have
//! stays unused until edge support arrives.

use super::{Armed, prepare_pad};
use crate::{
    gpio::Level,
    peripherals::LP_AON,
    rtc_cntl::{
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
        let bit = 1 << pin.lp;

        pads |= bit;
        if pin.level == Level::High {
            levels |= bit;
        }

        prepare_pad(pin, kind);
    }

    write_ext1(pads, levels);
    WakeupSource::Ext1.enable();
}

fn write_ext1(pads: u8, levels: u8) {
    LP_AON::regs()
        .ext_wakeup_cntl()
        .modify(|_, w| w.ext_wakeup_status_clr().set_bit());
    LP_AON::regs().ext_wakeup_cntl().modify(|_, w| unsafe {
        w.ext_wakeup_status_clr().clear_bit();
        w.ext_wakeup_sel().bits(pads);
        w.ext_wakeup_lv().bits(levels)
    });
}

//! The low-power path of esp32c5, esp32c6, esp32c61 and esp32h2.
//!
//! On these chips, `ext1` has one level per pad, covers every low-power pad, and does not increase
//! the sleep current. There is thus nothing to compare, and every pin goes to `ext1`. These chips
//! also have a per-pin path, which stays unused until esp-hal supports edge triggers.

use super::{Armed, prepare_pad};
use crate::{
    gpio::Level,
    peripherals::LP_AON,
    rtc_cntl::{
        WakeupReason,
        WakeupSource,
        sleep::{SleepKind, WrappedSleepConfig},
    },
};

/// Clears the paths that the pins of this chip can take, so that no bit of a previous sleep wakes
/// the chip.
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

/// Returns whether this pad ended the last sleep through `ext1`, which is the only low-power path
/// that these chips use.
pub(super) fn caused_wakeup(gpio: u8, cause: WakeupReason) -> bool {
    // Only a low-power pad can take this path, and `ext1` selects it by its low-power number.
    let Some(lp) = super::lp_number(gpio) else {
        return false;
    };

    cause.contains(WakeupSource::Ext1)
        && LP_AON::regs()
            .ext_wakeup_cntl()
            .read()
            .ext_wakeup_status()
            .bits()
            & (1 << lp)
            != 0
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

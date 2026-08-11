//! The low-power paths of esp32, esp32s2 and esp32s3.
//!
//! On these chips, `ext1` has one level for the complete pad mask. It can therefore serve only the
//! pins that wake on the same level. It is also the only low-power path that keeps the low-power
//! peripheral domain powered down. It thus takes the largest group of pins with the same level, and
//! the remaining pins keep that domain powered. One remaining pin goes to `ext0`, which takes a
//! single pad with its own level. All further pins go to the per-pin path, which gives each
//! low-power pad its own level.

use super::{Armed, LP_NUMBERS, NO_LP_NUMBER, prepare_pad};
use crate::{
    gpio::{Level, lp_io::low_level},
    peripherals::{LPWR, RTC_IO},
    rtc_cntl::{
        WakeupReason,
        WakeupSource,
        sleep::{SleepResource, WrappedSleepConfig},
    },
};

/// Clears the paths that the pins of this chip can take, so that no bit of a previous sleep wakes
/// the chip.
pub(super) fn disable() {
    WakeupSource::Ext0.disable();
    WakeupSource::Ext1.disable();
}

pub(super) fn allocate(armed: &[Armed], config: &mut WrappedSleepConfig<'_>) {
    clear_per_pin();

    let deep = config.is_deep_sleep();
    let shared = shared_level(armed);

    // Each armed pin gets a path. `ext1` takes one level group, `ext0` takes one remaining pin, and
    // the per-pin path takes all the others, because it covers every low-power pad. A chip with a
    // smaller per-pin path, or an allocation with fewer slots for edge triggers, breaks this rule.
    let mut leftover = false;
    let mut ext0_taken = false;
    for pin in armed {
        if Some(pin.level) == shared {
            continue;
        }

        leftover = true;
        if !ext0_taken {
            arm_ext0(pin, deep);
            ext0_taken = true;
        } else {
            arm_per_pin(pin, deep);
        }
    }

    if !ext0_taken {
        WakeupSource::Ext0.disable();
    }

    if leftover {
        // `ext0` and the per-pin path read the pad through the low-power IO peripheral.
        config.keep_alive(SleepResource::LpPeripherals);
    }

    match shared {
        Some(level) => {
            let group = |pin: &&Armed| pin.level == level;
            arm_ext1(armed.iter().filter(group), level, deep);
        }
        None => disarm_ext1(),
    }
}

/// Selects the level that `ext1` serves, or `None` if it can serve no pin.
///
/// The larger group wins, because each pin outside that group keeps the low-power peripheral domain
/// powered. If the two groups have the same size, the lower pin number wins. Two sleeps with the
/// same pins then draw the same current.
fn shared_level(armed: &[Armed]) -> Option<Level> {
    fn group(armed: &[Armed], level: Level) -> (usize, u8) {
        let mut count = 0;
        let mut lowest = u8::MAX;
        for pin in armed.iter().filter(|pin| pin.level == level) {
            count += 1;
            lowest = lowest.min(pin.gpio);
        }
        (count, lowest)
    }

    let high = group(armed, Level::High);
    let mut low = group(armed, Level::Low);

    // On esp32, `ext1` combines the selected pads with AND for a low level, and not with OR. A low
    // group therefore does what the user requested only if it holds a single pad.
    if cfg!(esp32) && low.0 > 1 {
        low = (0, u8::MAX);
    }

    match (high.0, low.0) {
        (0, 0) => None,
        (_, 0) => Some(Level::High),
        (0, _) => Some(Level::Low),
        _ if high.0 > low.0 => Some(Level::High),
        _ if low.0 > high.0 => Some(Level::Low),
        _ if high.1 < low.1 => Some(Level::High),
        _ => Some(Level::Low),
    }
}

/// Arms one level group on `ext1`.
fn arm_ext1<'a>(group: impl Iterator<Item = &'a Armed>, level: Level, deep: bool) {
    let mut pads = 0;
    for pin in group {
        pads |= 1 << pin.lp;
        prepare_pad(pin, deep);
    }

    write_ext1(pads, level);
    WakeupSource::Ext1.enable();
}

fn disarm_ext1() {
    write_ext1(0, Level::Low);
    WakeupSource::Ext1.disable();
}

fn write_ext1(pads: u32, level: Level) {
    LPWR::regs()
        .ext_wakeup1()
        .modify(|_, w| w.status_clr().set_bit());
    LPWR::regs()
        .ext_wakeup1()
        .modify(|_, w| unsafe { w.sel().bits(pads) });
    LPWR::regs()
        .ext_wakeup_conf()
        .modify(|_, w| w.ext_wakeup1_lv().bit(level == Level::High));
}

/// Arms one pad on `ext0`, which is the oldest of the low-power paths.
fn arm_ext0(pin: &Armed, deep: bool) {
    prepare_pad(pin, deep);

    RTC_IO::regs()
        .ext_wakeup0()
        .modify(|_, w| unsafe { w.sel().bits(pin.lp) });
    LPWR::regs()
        .ext_wakeup_conf()
        .modify(|_, w| w.ext_wakeup0_lv().bit(pin.level == Level::High));

    WakeupSource::Ext0.enable();
}

/// Returns whether this pad ended the last sleep through one of the three low-power paths.
///
/// Only `ext0` has no status of its own. For that path, the result comes from the pad that the
/// sleep armed, because the sleep does not clear the selection.
pub(super) fn caused_wakeup(gpio: u8, cause: WakeupReason) -> bool {
    // Only a low-power pad can take one of these paths.
    let Some(lp) = super::lp_number(gpio) else {
        return false;
    };

    let ext1 = cause.contains(WakeupSource::Ext1)
        && LPWR::regs()
            .ext_wakeup1_status()
            .read()
            .ext_wakeup1_status()
            .bits()
            & (1 << lp)
            != 0;

    let ext0 = cause.contains(WakeupSource::Ext0)
        && RTC_IO::regs().ext_wakeup0().read().sel().bits() == lp;

    // The per-pin path uses the same wakeup source as the digital path, but it has its own status.
    let per_pin = cause.contains(WakeupSource::Gpio) && low_level::wakeup_status() & (1 << lp) != 0;

    ext1 || ext0 || per_pin
}

/// Releases the pads that the previous run armed for a deep sleep, on all three paths.
///
/// Each path keeps the pads that it armed through the wake, so the registers still name them here.
/// `ext0` has one pad and no enable of its own, so its selection counts only while the source is
/// enabled.
pub(super) fn wake_io_reset() {
    let ext1 = LPWR::regs().ext_wakeup1().read().sel().bits();
    let ext0 = RTC_IO::regs().ext_wakeup0().read().sel().bits();
    let ext0_armed = crate::rtc_cntl::sleep::enabled_sources().contains(WakeupSource::Ext0);
    let per_pin = low_level::wakeup_enabled_mask();

    for lp in super::low_power_numbers() {
        if ext1 & (1 << lp) != 0 || (ext0_armed && ext0 == lp) || per_pin & (1 << lp) != 0 {
            low_level::pad_hold(lp, false);
        }
    }
}

/// Disarms the per-pin path of every pad, so that a pad outside this sleep cannot wake the chip.
fn clear_per_pin() {
    for &lp in LP_NUMBERS.iter().filter(|&&lp| lp != NO_LP_NUMBER) {
        low_level::apply_wakeup(lp, false, Level::Low);
    }

    // A status bit from an earlier sleep names the wrong pad as the pad that ended this sleep.
    low_level::clear_wakeup_status();
}

fn arm_per_pin(pin: &Armed, deep: bool) {
    prepare_pad(pin, deep);
    low_level::apply_wakeup(pin.lp, true, pin.level);
}

//! The low-power paths of esp32, esp32s2 and esp32s3.
//!
//! `ext1` has a single level for the whole pad mask here, so it can only serve pins that wake on
//! the same level. It is also the only low-power path that keeps the low-power peripheral domain
//! powered down, so it takes the largest group of pins that share a level, and the rest pay for
//! the domain: one pin on `ext0`, which takes a single pad at a level of its own, and any further
//! pins on the per-pin path, which gives every low-power pad a level of its own.

use super::{Armed, LP_NUMBERS, prepare_pad};
use crate::{
    gpio::{Level, lp_io::low_level},
    peripherals::{LPWR, RTC_IO},
    rtc_cntl::{
        WakeupSource,
        sleep::{SleepKind, SleepResource, WrappedSleepConfig},
    },
};

/// Clears the paths that this chip's pins can take, so that a stale bit cannot wake the chip.
pub(super) fn disable() {
    WakeupSource::Ext0.disable();
    WakeupSource::Ext1.disable();
}

pub(super) fn allocate(armed: &[Armed], kind: SleepKind, config: &mut WrappedSleepConfig<'_>) {
    clear_per_pin();

    let shared = shared_level(armed);

    // Every armed pin gets a path: `ext1` takes one level group, `ext0` one of the leftovers, and
    // the per-pin path covers every low-power pad, so it takes the rest. A chip with a smaller
    // per-pin path, or an edge-capable allocation with fewer slots, would break that.
    let mut leftover = false;
    let mut ext0_taken = false;
    for pin in armed {
        if Some(pin.level) == shared {
            continue;
        }

        leftover = true;
        if !ext0_taken {
            arm_ext0(pin, kind);
            ext0_taken = true;
        } else {
            arm_per_pin(pin, kind);
        }
    }

    if !ext0_taken {
        WakeupSource::Ext0.disable();
    }

    if leftover {
        // `ext0` and the per-pin path read the pad through the low-power IO pads.
        config.keep_alive(SleepResource::LpPeripherals);
    }

    match shared {
        Some(level) => {
            let group = |pin: &&Armed| pin.level == level;
            arm_ext1(armed.iter().filter(group), level, kind);
        }
        None => disarm_ext1(),
    }
}

/// Picks the level that `ext1` serves, or `None` if it cannot serve any pin.
///
/// The larger group wins, because every pin outside it costs the low-power peripheral domain. When
/// the groups are the same size the lower pin number wins, so that two otherwise identical sleeps
/// draw the same current.
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

    // esp32's low condition is an AND across the selected pads rather than an OR, so a low group
    // only means what the user asked for while it holds a single pad.
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
fn arm_ext1<'a>(group: impl Iterator<Item = &'a Armed>, level: Level, kind: SleepKind) {
    let mut pads = 0;
    for pin in group {
        pads |= 1 << pin.lp;
        prepare_pad(pin, kind);
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

/// Arms one pad on `ext0`, the oldest of the low-power paths.
fn arm_ext0(pin: &Armed, kind: SleepKind) {
    prepare_pad(pin, kind);

    RTC_IO::regs()
        .ext_wakeup0()
        .modify(|_, w| unsafe { w.sel().bits(pin.lp) });
    LPWR::regs()
        .ext_wakeup_conf()
        .modify(|_, w| w.ext_wakeup0_lv().bit(pin.level == Level::High));

    WakeupSource::Ext0.enable();
}

/// Disarms every pad's per-pin path, so that the pads this sleep did not choose cannot wake it.
fn clear_per_pin() {
    for &lp in LP_NUMBERS.iter().filter(|&n| n != 0xFF) {
        low_level::apply_wakeup(lp, false, Level::Low);
    }
}

fn arm_per_pin(pin: &Armed, kind: SleepKind) {
    prepare_pad(pin, kind);
    low_level::apply_wakeup(pin.lp, true, pin.level);
}

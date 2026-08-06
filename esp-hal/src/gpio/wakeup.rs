//! Waking the chip through a pad.
//!
//! A pin declares that it may wake the chip, and the interrupt configuration says on what. Sleep
//! entry then assigns the pins that are listening to the hardware paths the chip has, with the
//! whole set in view, so the assignment does not depend on the order the pins were configured in.
//!
//! There are two kinds of path. The digital path reads the pad through the high-performance GPIO
//! peripheral, works on every pad, and needs that peripheral powered. Every listening pin gets it,
//! because the pin's interrupt-enable and wakeup-enable bits are written together. The low-power
//! paths — `ext0`, `ext1` and the per-pin low-power path — survive the peripheral's power-down, so
//! they are what deep sleep, and a light sleep that powers the peripheral down, need. Only
//! low-power pads have them, and a pin has to ask for them through [`WakeupConfig`].
//!
//! Because it owns the pad tables, this module also holds the deep-sleep pad isolation, which is a
//! sleep-entry step rather than a wakeup source.

use portable_atomic::{AtomicU64, Ordering};

use crate::{
    gpio::{AnyPin, Event, Level, LpPin, Pin, WakeConfigError, lp_io::LpFunction},
    peripherals::GPIO,
    rtc_cntl::{
        WakeupSource,
        sleep::{SleepKind, SleepResource, WrappedSleepConfig},
    },
};

/// Configures whether a pin may wake the chip from sleep.
///
/// What wakes the chip is the pin's interrupt trigger, set by
/// [`listen`][crate::gpio::Input::listen] or by the `wait_for` family: **a pin that is not
/// listening is not a wakeup source**, and a pin that listens wakes the chip from light sleep
/// through the digital path without any configuration.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct WakeupConfig {
    /// Lets the pin wake the chip while the high-performance GPIO peripheral is powered down.
    ///
    /// This is what deep sleep needs, and what a light sleep that powers the peripheral down
    /// needs. Only low-power pads have such a path.
    ///
    /// Sleep entry switches the pad to the low-power IO MUX and enables its low-power input, and,
    /// for a deep sleep, holds the pad. A light sleep gives the pad back to the digital GPIO
    /// peripheral when it ends; a deep sleep releases the hold when the chip boots again.
    low_power_path: bool,
}

/// The pads that may wake the chip through a low-power path.
///
/// The digital path is recorded in hardware, in the pad's own wakeup-enable bit, but no low-power
/// register means "may wake" without also arming a level, and esp32h2 has no per-pin low-power
/// register at all. This record is therefore software. It does not have to survive a deep sleep,
/// because that wake resets the chip, and a program starts with no wakeup sources.
static LOW_POWER_PADS: AtomicU64 = AtomicU64::new(0);

/// The pads that sleep entry gave to the low-power IO MUX, and that a light sleep has to give
/// back.
static PREPARED_PADS: AtomicU64 = AtomicU64::new(0);

for_each_lp_function! {
    (LP_GPIOn $( (($_sig:ident, LP_GPIOn, $lp:literal), $gpio:ident, $_af:ident, $_in:tt $_out:tt) ),*) => {
        /// The low-power pads: the digital pin number, and the number the low-power registers
        /// index the pad by.
        const LOW_POWER_PADS_TABLE: &[(u8, u8)] = &[
            $( (crate::peripherals::$gpio::NUMBER, $lp) ),*
        ];
    };
}

for_each_gpio! {
    (all $( ($n:literal, $gpio:ident $_ins:tt $_outs:tt $_attrs:tt) ),*) => {
        /// Every pad, so that the digital path can be read back from hardware.
        const PADS: &[u8] = &[ $( $n ),* ];
    };
}

const MAX_ARMED: usize = LOW_POWER_PADS_TABLE.len();

/// A listening pin, and the level that wakes the chip.
#[derive(Debug, Clone, Copy)]
struct Armed {
    /// The digital pin number.
    gpio: u8,
    /// The number the low-power registers index the pad by.
    // esp32c2 and esp32c3 index their only low-power path by digital pin number instead.
    #[cfg_attr(not(sleep_ext1_version_is_set), expect(dead_code))]
    lp: u8,
    level: Level,
}

impl Armed {
    const NONE: Self = Self {
        gpio: 0,
        lp: 0,
        level: Level::Low,
    };
}

/// Applies a pin's wakeup configuration.
pub(crate) fn apply_config(pin: &AnyPin<'_>, config: &WakeupConfig) -> Result<(), WakeConfigError> {
    let pad = 1 << pin.number();

    if config.low_power_path {
        if lp_number(pin.number()).is_none() {
            return Err(WakeConfigError::NoLowPowerPath);
        }

        LOW_POWER_PADS.fetch_or(pad, Ordering::Relaxed);
        enable();
    } else {
        LOW_POWER_PADS.fetch_and(!pad, Ordering::Relaxed);
    }

    Ok(())
}

/// Records that a pad may wake the chip, and registers the hooks that allocate the paths.
///
/// The mask bit is inert on its own: waking also needs the digital per-pin bit that `listen`
/// writes, or a low-power path that the entry hook assigns. The entry hook clears the bit again
/// when it finds that no pin can wake the chip, so that a sleep with no other wakeup source is
/// refused rather than never ending.
pub(crate) fn enable() {
    WakeupSource::Gpio.enable_with_hooks(Some(entry_hook), Some(exit_hook));
}

fn disable() {
    WakeupSource::Gpio.disable();

    // A stale `ext0` or `ext1` bit would outlive the hook that cleans it up, and the chip would
    // carry a wakeup source nobody asked for.
    #[cfg(sleep_has_wakeup_source_ext0)]
    WakeupSource::Ext0.disable();
    #[cfg(sleep_has_wakeup_source_ext1)]
    WakeupSource::Ext1.disable();
}

fn lp_number(gpio: u8) -> Option<u8> {
    LOW_POWER_PADS_TABLE
        .iter()
        .find(|(pad, _)| *pad == gpio)
        .map(|(_, lp)| *lp)
}

/// Assigns the listening pins to the hardware paths, and votes for what they need powered.
#[crate::ram]
fn entry_hook(kind: SleepKind, config: &mut WrappedSleepConfig<'_>) {
    let mut buffer = [Armed::NONE; MAX_ARMED];
    let (armed, mut digital) = collect(&mut buffer);

    // Deep sleep powers the high-performance GPIO peripheral down, so the digital path cannot wake
    // the chip from it however many pins are listening.
    digital &= kind == SleepKind::Light;

    if armed.is_empty() && !digital {
        // Nothing can wake the chip through a pad, so the record has to go: a mask that claims
        // otherwise would let a sleep with no other wakeup source through.
        disable();
        return;
    }

    if digital {
        // The digital path reads the pad through the high-performance GPIO peripheral.
        config.keep_alive(SleepResource::HpPeripherals);
    }

    allocate(armed, kind, config);
}

/// Gives the pads back to the digital GPIO peripheral after a light sleep.
#[crate::ram]
fn exit_hook() {
    let prepared = PREPARED_PADS.swap(0, Ordering::Relaxed);

    for &(gpio, _) in LOW_POWER_PADS_TABLE {
        if prepared & (1 << gpio) == 0 {
            continue;
        }

        let pad = unsafe { AnyPin::steal(gpio) };
        pad.lp_pad_hold(false);
        pad.lp_set_config(true, false, LpFunction::LP_GPIO);
    }
}

/// Collects the listening pins that asked for a low-power path, and reports whether any pin at all
/// is listening.
#[crate::ram]
fn collect(buffer: &mut [Armed; MAX_ARMED]) -> (&[Armed], bool) {
    let participants = LOW_POWER_PADS.load(Ordering::Relaxed);

    let mut count = 0;
    for &(gpio, lp) in LOW_POWER_PADS_TABLE {
        if participants & (1 << gpio) == 0 {
            continue;
        }
        if let Some(level) = armed_level(gpio) {
            buffer[count] = Armed { gpio, lp, level };
            count += 1;
        }
    }

    let digital = PADS.iter().any(|&gpio| {
        GPIO::regs()
            .pin(gpio as usize)
            .read()
            .wakeup_enable()
            .bit_is_set()
    });

    (&buffer[..count], digital)
}

/// Returns the level that wakes the chip through this pad, or `None` if the pin is not listening.
///
/// An edge trigger becomes the level the edge ends on, so that the user's intent survives on the
/// chips whose wake paths are level-only. `AnyEdge` ends on the level the pin is not at now, which
/// is what "wake when this changes" means. Sampling it races with the pin, benignly: a pin that
/// changes before the sleep starts arms a level that is already asserted, so the sleep is rejected
/// or ends at once.
#[crate::ram]
fn armed_level(gpio: u8) -> Option<Level> {
    let pin = GPIO::regs().pin(gpio as usize).read();

    if pin.int_ena().bits() == 0 {
        return None;
    }

    let trigger = pin.int_type().bits();
    Some(
        if trigger == Event::HighLevel as u8 || trigger == Event::RisingEdge as u8 {
            Level::High
        } else if trigger == Event::LowLevel as u8 || trigger == Event::FallingEdge as u8 {
            Level::Low
        } else if trigger == Event::AnyEdge as u8 {
            !Level::from(unsafe { AnyPin::steal(gpio) }.is_input_high())
        } else {
            return None;
        },
    )
}

/// Every chip whose `ext1` has per-pin levels: esp32c5, esp32c6, esp32c61, esp32h2 and esp32p4.
///
/// `ext1` covers every low-power pad at no cost in sleep current, so there is nothing to weigh: all
/// pins go there, and the per-pin path stays unused until edge support arrives.
#[cfg(any(sleep_ext1_version = "2", sleep_ext1_version = "3"))]
#[crate::ram]
fn allocate(armed: &[Armed], kind: SleepKind, _config: &mut WrappedSleepConfig<'_>) {
    arm_ext1(armed, kind);
}

/// esp32c2 and esp32c3, which have no `ext1` and no low-power peripheral domain to keep alive.
///
/// The per-pin path is the only low-power path, and it is free, so there is nothing to allocate.
#[cfg(all(not(sleep_ext1_version_is_set), sleep_pin_wakeup_version_is_set))]
#[crate::ram]
fn allocate(armed: &[Armed], kind: SleepKind, _config: &mut WrappedSleepConfig<'_>) {
    clear_per_pin();

    for pin in armed {
        arm_per_pin(pin, kind);
    }

    prepare_gpio_wakeup();
}

/// esp32, esp32s2 and esp32s3, whose `ext1` has one level for the whole pad mask.
///
/// `ext1` is the only low-power path that keeps the low-power peripheral domain powered down, so it
/// takes the largest group of pins that share a level, and the rest pay for the domain: one pin on
/// `ext0`, and any further pins on the per-pin path.
#[cfg(sleep_ext1_version = "1")]
#[crate::ram]
fn allocate(armed: &[Armed], kind: SleepKind, config: &mut WrappedSleepConfig<'_>) {
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
            arm_ext0(pin);
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
            arm_ext1_group(armed.iter().filter(group), level, kind);
        }
        None => disarm_ext1(),
    }
}

/// Picks the level that `ext1` serves, or `None` if it cannot serve any pin.
///
/// The larger group wins, because every pin outside it costs the low-power peripheral domain. When
/// the groups are the same size the lower pin number wins, so that two otherwise identical sleeps
/// draw the same current.
#[cfg(sleep_ext1_version = "1")]
#[crate::ram]
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

/// Arms the whole set on `ext1`, which has a level per pad here.
#[cfg(any(sleep_ext1_version = "2", sleep_ext1_version = "3"))]
#[crate::ram]
fn arm_ext1(armed: &[Armed], kind: SleepKind) {
    if armed.is_empty() {
        disarm_ext1();
        return;
    }

    let mut pads = 0;
    let mut levels = 0;
    for pin in armed {
        // esp32p4 selects the pads by digital pin number, the others by low-power number.
        let bit = 1
            << if cfg!(sleep_ext1_version = "3") {
                pin.gpio
            } else {
                pin.lp
            };

        pads |= bit;
        if pin.level == Level::High {
            levels |= bit;
        }

        prepare_pad(pin.gpio, kind);
    }

    write_ext1(pads, levels);
    WakeupSource::Ext1.enable();
}

/// Arms one level group on `ext1`, which has a single level for the whole pad mask here.
#[cfg(sleep_ext1_version = "1")]
#[crate::ram]
fn arm_ext1_group<'a>(group: impl Iterator<Item = &'a Armed>, level: Level, kind: SleepKind) {
    let mut pads = 0;
    for pin in group {
        pads |= 1 << pin.lp;
        prepare_pad(pin.gpio, kind);
    }

    write_ext1(pads, level);
    WakeupSource::Ext1.enable();
}

#[cfg(sleep_ext1_version_is_set)]
#[crate::ram]
fn disarm_ext1() {
    cfg_select! {
        sleep_ext1_version = "1" => write_ext1(0, Level::Low),
        _ => write_ext1(0, 0),
    }
    WakeupSource::Ext1.disable();
}

#[cfg(sleep_ext1_version = "1")]
#[crate::ram]
fn write_ext1(pads: u32, level: Level) {
    use crate::peripherals::LPWR;

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

#[cfg(sleep_ext1_version = "2")]
#[crate::ram]
fn write_ext1(pads: u8, levels: u8) {
    use crate::peripherals::LP_AON;

    LP_AON::regs()
        .ext_wakeup_cntl()
        .modify(|_, w| w.ext_wakeup_status_clr().set_bit());
    LP_AON::regs().ext_wakeup_cntl().modify(|_, w| unsafe {
        w.ext_wakeup_status_clr().clear_bit();
        w.ext_wakeup_sel().bits(pads);
        w.ext_wakeup_lv().bits(levels)
    });
}

#[cfg(sleep_ext1_version = "3")]
#[crate::ram]
fn write_ext1(pads: u32, levels: u32) {
    use crate::peripherals::PMU;

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

/// Arms one pad on `ext0`, the oldest of the low-power paths, which takes a single pad at a level
/// of its own.
#[cfg(sleep_has_wakeup_source_ext0)]
#[crate::ram]
fn arm_ext0(pin: &Armed) {
    use crate::peripherals::{LPWR, RTC_IO};

    unsafe { AnyPin::steal(pin.gpio) }.lp_set_config(true, true, LpFunction::LP_GPIO);

    RTC_IO::regs()
        .ext_wakeup0()
        .modify(|_, w| unsafe { w.sel().bits(pin.lp) });
    LPWR::regs()
        .ext_wakeup_conf()
        .modify(|_, w| w.ext_wakeup0_lv().bit(pin.level == Level::High));

    WakeupSource::Ext0.enable();
}

/// Disarms every pad's per-pin path, so that the pads this sleep did not choose cannot wake it.
#[cfg(all(
    sleep_pin_wakeup_version_is_set,
    any(sleep_ext1_version = "1", not(sleep_ext1_version_is_set))
))]
#[crate::ram]
fn clear_per_pin() {
    for &(gpio, _) in LOW_POWER_PADS_TABLE {
        unsafe { AnyPin::steal(gpio) }.apply_wakeup(false, Level::Low);
    }
}

/// Arms one pad on the per-pin low-power path, which gives every low-power pad a level of its own.
#[cfg(all(
    sleep_pin_wakeup_version_is_set,
    any(sleep_ext1_version = "1", not(sleep_ext1_version_is_set))
))]
#[crate::ram]
fn arm_per_pin(pin: &Armed, kind: SleepKind) {
    let pad = unsafe { AnyPin::steal(pin.gpio) };

    cfg_select! {
        sleep_pin_wakeup_version = "2" => {
            {
                // esp32c2 and esp32c3 reach the pad through the digital IO MUX, so there is no
                // low-power function to select. ESP-IDF drives the pad to its wake level with the
                // pull resistors, so that a deep sleep, which isolates the pad, cannot lose it.
                if kind == SleepKind::Deep {
                    use crate::gpio::LpPinWithResistors;
                    pad.lp_pullup(pin.level == Level::Low);
                    pad.lp_pulldown(pin.level == Level::High);
                }
            }
        }
        _ => {
            prepare_pad(pin.gpio, kind);
        }
    }

    pad.apply_wakeup(true, pin.level);
}

/// Gives the pad to the low-power IO MUX, which is what makes it readable while the
/// high-performance GPIO peripheral is powered down.
///
/// A deep sleep also holds the pad, because it powers down whatever drives it. A light sleep does
/// not: nothing it powers down drives the pad, and the hold would freeze an output.
#[cfg(any(sleep_ext1_version_is_set, sleep_pin_wakeup_version = "1"))]
#[crate::ram]
fn prepare_pad(gpio: u8, kind: SleepKind) {
    let pad = unsafe { AnyPin::steal(gpio) };

    // esp32h2 reaches the pad through the digital IO MUX, so it has no low-power mux to switch.
    pad.lp_set_config(true, !cfg!(esp32h2), LpFunction::LP_GPIO);
    pad.lp_pad_hold(kind == SleepKind::Deep);

    PREPARED_PADS.fetch_or(1 << gpio, Ordering::Relaxed);
}

/// Clocks the pad-scanning logic that the esp32c2 and esp32c3 per-pin path needs, and clears the
/// status a previous wake left behind.
#[cfg(sleep_pin_wakeup_version = "2")]
#[crate::ram]
fn prepare_gpio_wakeup() {
    use crate::peripherals::LPWR;

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

/// Cuts the pads that are not held loose from the digital peripheral, so that they do not raise
/// the deep-sleep current through a powered-down output driver.
///
/// This is not a property of any wakeup source, so every deep sleep does it, the way ESP-IDF's
/// `esp_sleep_isolate_digital_gpio` does. It is destructive — a pad that a peripheral was driving
/// loses that function — which is why it runs at sleep entry, on the chips that cannot hold a
/// single pad through a deep sleep, and after the wakeup sources have taken the holds they need.
#[cfg(sleep_deep_sleep_needs_gpio_isolation)]
#[crate::ram]
pub(crate) fn isolate_pads_for_deep_sleep() {
    use crate::{
        gpio::{AlternateFunction, OutputSignal, io_mux_reg},
        peripherals::LPWR,
    };

    // With the hold disabled, no pad keeps its level through the sleep anyway, and ESP-IDF skips
    // the isolation for the same reason.
    let dig_iso = LPWR::regs().dig_iso().read();
    let hold_enabled = !dig_iso.dg_pad_force_unhold().bit() && dig_iso.dg_pad_autohold_en().bit();
    if !hold_enabled {
        return;
    }

    let held = cfg_select! {
        esp32 => crate::peripherals::RTC_IO::regs()
            .dig_pad_hold()
            .read()
            .bits(),
        _ => LPWR::regs().dig_pad_hold().read().bits(),
    };

    for &gpio in PADS {
        // Only the pads the digital supply feeds leak here, and they are exactly the pads with no
        // low-power number. The low-power pads have their own supply, which stays up, and a wake
        // pad is always one of them.
        if lp_number(gpio).is_some() {
            continue;
        }

        // A pad the user holds keeps the level it was left at, which is the point of holding it,
        // so isolating it would undo the request.
        if held & digital_hold_mask(gpio) != 0 {
            continue;
        }

        GPIO::regs()
            .func_out_sel_cfg(gpio as usize)
            .modify(|_, w| unsafe { w.bits(OutputSignal::GPIO as u32) });

        io_mux_reg(gpio).modify(|_, w| unsafe {
            w.fun_wpu().clear_bit();
            w.fun_wpd().clear_bit();
            w.fun_ie().clear_bit();
            // A pad left on a peripheral function raises the deep-sleep current.
            w.mcu_sel().bits(AlternateFunction::GPIO as u8)
        });
    }
}

/// The bit that holds `gpio` in the digital pad-hold register.
///
/// The register covers the digital pads only, so the bit is not the pin number on every chip: the
/// esp32s2 and esp32s3 registers start at the first digital pad, and the esp32 register packs the
/// pads it has in an order of its own.
#[cfg(sleep_deep_sleep_needs_gpio_isolation)]
#[crate::ram]
fn digital_hold_mask(gpio: u8) -> u32 {
    cfg_select! {
        esp32 => {
            1 << match gpio {
                1 => 1,
                3 => 0,
                5 => 8,
                6..=11 => gpio - 4,
                16..=19 | 21..=23 => gpio - 7,
                // No other pad is fed by the digital supply, so none is reached here.
                _ => return 0,
            }
        }
        any(esp32s2, esp32s3) => 1 << (gpio - 21),
        _ => 1 << gpio,
    }
}

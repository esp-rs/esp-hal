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
//! Which low-power paths a chip has, and how they divide the pins between them, is what the
//! [`path`] module holds: this module collects the pins and votes for what they need powered.
//!
//! Because it owns the pad tables, this module also holds the deep-sleep pad isolation, which is a
//! sleep-entry step rather than a wakeup source.
//!
//! Nothing here runs with the flash inaccessible: sleep entry calls the hooks before it hands the
//! configuration to hardware, and calls the exit hook after the wake sequence has restored it. The
//! code therefore stays in flash, where the compiler may inline it.

// The low-power path is what the version decides, so each generation brings its own allocation.
#[cfg_attr(sleep_ext1_version = "1", path = "ext1_v1.rs")]
#[cfg_attr(sleep_ext1_version = "2", path = "ext1_v2.rs")]
#[cfg_attr(sleep_ext1_version = "3", path = "ext1_v3.rs")]
#[cfg_attr(not(sleep_ext1_version_is_set), path = "per_pin.rs")]
mod path;

use portable_atomic::{AtomicU32, Ordering};
use strum::EnumCount;

use crate::{
    gpio::{
        AnyPin,
        Event,
        GpioBank,
        Level,
        Pin,
        WakeConfigError,
        low_level::bank,
        lp_io::{LpFunction, low_level},
    },
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
///
/// # Holding the pad at the level that does not wake the chip
///
/// A pad that is at its wake level when the sleep starts ends the sleep at once, so something has
/// to hold it at the other level. That something is the pin's own configuration: sleep keeps the
/// pull resistors the pin is configured with, and adds none of its own. A pin left with
/// [`Pull::None`][crate::gpio::Pull::None] and no external resistor therefore floats through the
/// sleep, and a floating pad wakes the chip immediately and every time.
///
/// So give a level-triggered wake pin a pull against the level it wakes on, or an external
/// resistor. An external pull-up is the better choice for a pin that wakes the chip on a low
/// level.
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
    /// Sleep entry gives the pad whatever the low-power path needs — the pin's pull resistors stay
    /// in force either way — and, for a deep sleep, holds the pad, because deep sleep powers down
    /// whatever drives it. A light sleep gives the pad back to the digital GPIO peripheral when it
    /// ends; a deep sleep releases the hold when the chip boots again.
    low_power_path: bool,
}

/// A set of pads, held in the words the GPIO peripheral groups its pins into.
struct PadMask([AtomicU32; GpioBank::COUNT]);

impl PadMask {
    const fn new() -> Self {
        Self([const { AtomicU32::new(0) }; GpioBank::COUNT])
    }

    fn word(&self, bank: GpioBank) -> &AtomicU32 {
        &self.0[bank as usize]
    }

    fn set(&self, gpio: u8, member: bool) {
        let bank = bank(gpio);
        let pin = 1 << (gpio - bank.offset());

        if member {
            self.word(bank).fetch_or(pin, Ordering::Relaxed);
        } else {
            self.word(bank).fetch_and(!pin, Ordering::Relaxed);
        }
    }
}

/// The pads that may wake the chip through a low-power path.
///
/// The digital path is recorded in hardware, in the pad's own wakeup-enable bit, but no low-power
/// register means "may wake" without also arming a level, and esp32h2 has no per-pin low-power
/// register at all. This record is therefore software. It does not have to survive a deep sleep,
/// because that wake resets the chip, and a program starts with no wakeup sources.
static LOW_POWER_PADS: PadMask = PadMask::new();

/// The pads that sleep entry gave to the low-power IO MUX, and that a light sleep has to give
/// back.
static PREPARED_PADS: PadMask = PadMask::new();

for_each_gpio! {
    (all $( ($n:literal, $gpio:ident $_ins:tt $_outs:tt $_attrs:tt) ),*) => {
        /// Every pad, so that deep-sleep isolation can walk them.
        #[cfg(sleep_deep_sleep_needs_gpio_isolation)]
        const PADS: &[u8] = &[ $( $n ),* ];

        /// One past the highest pin number, which is what the pad tables are indexed by.
        const PAD_COUNT: usize = {
            let mut highest = 0;
            $( if $n > highest { highest = $n; } )*
            highest + 1
        };
    };
}

/// Marks a pad the low-power registers do not reach, in [`LP_NUMBERS`].
const NO_LP_NUMBER: u8 = u8::MAX;

for_each_lp_function! {
    (LP_GPIOn $( (($_sig:ident, LP_GPIOn, $lp:literal), $gpio:ident, $_af:ident, $_in:tt $_out:tt) ),*) => {
        /// The number the low-power registers index each pad by, for the pads they reach.
        ///
        /// The two domains number the pads separately, and only some chips give a pad the same
        /// number in both, so a low-power register takes the number from here and never the pin
        /// number.
        const LP_NUMBERS: [u8; PAD_COUNT] = {
            let mut numbers = [NO_LP_NUMBER; PAD_COUNT];
            $( numbers[crate::peripherals::$gpio::NUMBER as usize] = $lp; )*
            numbers
        };
    };
}

/// Every pin can be armed at once, so the set of listening pins never outgrows this.
const MAX_ARMED: usize = {
    let mut count = 0;
    let mut pad = 0;
    while pad < PAD_COUNT {
        if LP_NUMBERS[pad] != NO_LP_NUMBER {
            count += 1;
        }
        pad += 1;
    }
    count
};

/// A listening pin, and the level that wakes the chip.
#[derive(Debug, Clone, Copy)]
struct Armed {
    /// The digital pin number.
    // esp32c2 and esp32c3 keep the pad on the digital IO MUX, so nothing in their path needs it.
    #[cfg_attr(not(sleep_ext1_version_is_set), expect(dead_code))]
    gpio: u8,
    /// The number the low-power registers index the pad by.
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
    if config.low_power_path {
        if lp_number(pin.number()).is_none() {
            return Err(WakeConfigError::NoLowPowerPath);
        }

        LOW_POWER_PADS.set(pin.number(), true);
        enable();
    } else {
        LOW_POWER_PADS.set(pin.number(), false);
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

    // A stale bit of a path this chip has would outlive the hook that cleans it up, and the chip
    // would carry a wakeup source nobody asked for.
    path::disable();
}

/// The number the low-power registers index `gpio` by, or `None` if they do not reach the pad.
fn lp_number(gpio: u8) -> Option<u8> {
    match LP_NUMBERS[gpio as usize] {
        NO_LP_NUMBER => None,
        lp => Some(lp),
    }
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

    path::allocate(armed, kind, config);
}

/// Gives the pads back to the digital GPIO peripheral after a light sleep.
#[crate::ram]
fn exit_hook() {
    for bank in GpioBank::ALL {
        let mut prepared = PREPARED_PADS.word(bank).swap(0, Ordering::Relaxed);

        while prepared != 0 {
            let pin = prepared.trailing_zeros();
            prepared &= !(1 << pin);

            let Some(lp) = lp_number(bank.offset() + pin as u8) else {
                continue;
            };

            low_level::pad_hold(lp, false);
            low_level::set_config(lp, true, false, LpFunction::LP_GPIO);
        }
    }
}

/// Collects the listening pins that asked for a low-power path, and reports whether any pin at all
/// is listening.
///
/// A pin that listens also ends a light sleep, so the listening record answers both questions, and
/// neither one needs a walk over the pads.
fn collect(buffer: &mut [Armed; MAX_ARMED]) -> (&[Armed], bool) {
    let mut count = 0;
    let mut digital = false;

    for bank in GpioBank::ALL {
        let listening = bank.listening().load(Ordering::Relaxed);
        digital |= listening != 0;

        let mut participants = listening & LOW_POWER_PADS.word(bank).load(Ordering::Relaxed);
        while participants != 0 {
            let pin = participants.trailing_zeros();
            participants &= !(1 << pin);

            // Only a pad the low-power registers reach enters the mask, so the number is there.
            let gpio = bank.offset() + pin as u8;
            let Some(lp) = lp_number(gpio) else { continue };

            if let Some(level) = armed_level(gpio) {
                buffer[count] = Armed { gpio, lp, level };
                count += 1;
            }
        }
    }

    (&buffer[..count], digital)
}

/// Returns the level that wakes the chip through this pad, or `None` if its trigger names none.
///
/// An edge trigger becomes the level the edge ends on, so that the user's intent survives on the
/// chips whose wake paths are level-only. `AnyEdge` ends on the level the pin is not at now, which
/// is what "wake when this changes" means. Sampling it races with the pin, benignly: a pin that
/// changes before the sleep starts arms a level that is already asserted, so the sleep is rejected
/// or ends at once.
fn armed_level(gpio: u8) -> Option<Level> {
    let trigger = GPIO::regs().pin(gpio as usize).read().int_type().bits();
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

/// Gives the pad to the low-power IO MUX, which is what makes it readable while the
/// high-performance GPIO peripheral is powered down.
///
/// A deep sleep also holds the pad, because it powers down whatever drives it. A light sleep does
/// not: nothing it powers down drives the pad, and the hold would freeze an output.
#[cfg(sleep_ext1_version_is_set)]
fn prepare_pad(pin: &Armed, kind: SleepKind) {
    // The low-power IO MUX has pull resistors of its own, and the digital ones stop working the
    // moment the pad changes hands. Carry them over, or a pad that nothing drives floats away from
    // the level the user pulled it to and wakes the chip at once.
    #[cfg(not(esp32h2))]
    {
        let digital = crate::gpio::io_mux_reg(pin.gpio).read();
        low_level::pullup_enable(pin.lp, digital.fun_wpu().bit());
        low_level::pulldown_enable(pin.lp, digital.fun_wpd().bit());
    }

    // esp32h2 reaches the pad through the digital IO MUX, so it has no low-power mux to switch.
    low_level::set_config(pin.lp, true, !cfg!(esp32h2), LpFunction::LP_GPIO);
    low_level::pad_hold(pin.lp, kind == SleepKind::Deep);

    PREPARED_PADS.set(pin.gpio, true);
}

/// Cuts the pads that are not held loose from the digital peripheral, so that they do not raise
/// the deep-sleep current through a powered-down output driver.
///
/// This is not a property of any wakeup source, so every deep sleep does it, the way ESP-IDF's
/// `esp_sleep_isolate_digital_gpio` does. It is destructive — a pad that a peripheral was driving
/// loses that function — which is why it runs at sleep entry, on the chips that cannot hold a
/// single pad through a deep sleep, and after the wakeup sources have taken the holds they need.
#[cfg(sleep_deep_sleep_needs_gpio_isolation)]
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

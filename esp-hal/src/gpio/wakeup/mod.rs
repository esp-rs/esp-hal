//! Wake the chip through a pad.
//!
//! A pin declares that it can wake the chip, and its interrupt trigger sets the condition. Sleep
//! entry then assigns all the listening pins to the hardware paths of the chip. It assigns them
//! together, so the result does not depend on the order in which the user configured the pins.
//!
//! The chip has two kinds of path. The digital path reads the pad through the high-performance GPIO
//! peripheral. It works on every pad, but it needs that peripheral powered. Every listening pin has
//! this path, because the driver writes the interrupt-enable bit and the wakeup-enable bit of a pin
//! together. The low-power paths are `ext0`, `ext1` and the per-pin low-power path. These paths
//! continue to work when the peripheral is powered down. Deep sleep always powers the peripheral
//! down, and light sleep powers it down on request, so such a sleep needs a low-power path. Only
//! low-power pads have one, and a pin must request it with [`WakeupConfig`].
//!
//! The [`path`] module holds the low-power paths of the chip, and divides the pins between them.
//! This module collects the pins and requests the power domains that they need.
//!
//! This module also isolates the digital pads before a deep sleep, because it holds the pad tables.
//! That step is part of sleep entry, and it is not a wakeup source.
//!
//! All of this code runs with the flash accessible. Sleep entry calls the entry hook before it
//! writes the sleep configuration to hardware, and calls the exit hook after the wake sequence
//! restores it. The hooks still use the [`ram`][crate::ram] attribute, to keep the flash out of the
//! sleep path.

// Each generation of the low-power paths needs its own allocation.
#[cfg_attr(sleep_ext1_version = "1", path = "ext1_v1.rs")]
#[cfg_attr(sleep_ext1_version = "2", path = "ext1_v2.rs")]
#[cfg_attr(sleep_ext1_version = "3", path = "ext1_v3.rs")]
#[cfg_attr(not(sleep_ext1_version_is_set), path = "per_pin.rs")]
mod path;

use portable_atomic::Ordering;

// The pad function is selected only on the chips that have a low-power IO MUX.
#[cfg(sleep_ext1_version_is_set)]
use crate::gpio::lp_io::LpFunction;
use crate::{
    gpio::{
        AnyPin,
        Event,
        GpioBank,
        Level,
        Pin,
        WakeConfigError,
        low_level::PadMask,
        lp_io::low_level,
    },
    peripherals::GPIO,
    rtc_cntl::{
        WakeupSource,
        sleep::{SleepResource, WrappedSleepConfig},
    },
};

/// Configures whether a pin can wake the chip from sleep.
///
/// The wake condition is the interrupt trigger of the pin, which
/// [`listen`][crate::gpio::Input::listen] and the `wait_for` family set. **A pin that does not
/// listen is not a wakeup source.** A pin that listens wakes the chip from light sleep through the
/// digital path, and needs no configuration for that.
///
/// # Hold the pad at the level that does not wake the chip
///
/// A pad that is already at its wake level when the sleep starts ends the sleep immediately. The
/// configuration of the pin must hold the pad at the other level. Sleep keeps the pull resistors
/// that the pin is configured with, and adds no resistor of its own. A pin with
/// [`Pull::None`][crate::gpio::Pull::None] and no external resistor therefore floats during the
/// sleep, and a floating pad wakes the chip immediately and every time.
///
/// Give a level-triggered wake pin a pull against the level that wakes the chip, or an external
/// resistor. For a pin that wakes the chip on a low level, use an external pull-up.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub struct WakeupConfig {
    /// Lets the pin wake the chip while the high-performance GPIO peripheral is powered down.
    ///
    /// Deep sleep needs this path, and so does a light sleep that powers the peripheral down. Only
    /// low-power pads have such a path.
    ///
    /// Sleep entry configures the pad as the low-power path requires, and keeps the pull resistors
    /// of the pin in both cases. Before a deep sleep it also holds the pad, because deep sleep
    /// powers down the circuit that drives it. A light sleep returns the pad to the digital GPIO
    /// peripheral when it ends. After a deep sleep, the boot releases the hold.
    low_power_path: bool,
}

/// The pads that can wake the chip through a low-power path.
///
/// Hardware records the digital path, in the wakeup-enable bit of the pad. No low-power register
/// records the request alone, because every such register also arms a level, and esp32h2 has no
/// per-pin low-power register. This record is therefore in software. It does not have to survive a
/// deep sleep, because that wake resets the chip, and a program starts with no wakeup sources.
static LOW_POWER_PADS: PadMask = PadMask::new();

/// The pads that sleep entry prepared, and that the exit hook must return to their driver.
///
/// A sleep that ends without a reset has to undo the preparation. A light sleep is such a sleep,
/// and so is a deep sleep that the hardware rejects.
static PREPARED_PADS: PadMask = PadMask::new();

/// The pads that ended the last sleep.
///
/// Each path reports through a status that other code clears or overwrites later. The digital path
/// has no status of its own, only the interrupt of the pin, which the interrupt handler of the pin
/// clears. A low-power path keeps its status until the next sleep arms the path again.
/// [`record_wakeup`] therefore reads all the statuses when the sleep ends, and [`caused_wakeup`]
/// reads only this record.
static WOKEN_PADS: PadMask = PadMask::new();

for_each_gpio! {
    (all $( ($n:literal, $gpio:ident $_ins:tt $_outs:tt $_attrs:tt) ),*) => {
        /// The highest pin number plus one, which is the length of the pad tables.
        const PAD_COUNT: usize = {
            let mut highest = 0;
            $( if $n > highest { highest = $n; } )*
            highest + 1
        };
    };
}

/// Marks a pad that the low-power registers do not reach, in [`LP_NUMBERS`].
const NO_LP_NUMBER: u8 = u8::MAX;

for_each_lp_function! {
    (LP_GPIOn $( (($_sig:ident, LP_GPIOn, $lp:literal), $gpio:ident, $_af:ident, $_in:tt $_out:tt) ),*) => {
        /// The number that the low-power registers use for each pad that they reach.
        ///
        /// The two domains number the pads separately, and only some chips give a pad the same
        /// number in both domains. A low-power register therefore takes the number from this table,
        /// and never the pin number.
        const LP_NUMBERS: [u8; PAD_COUNT] = {
            let mut numbers = [NO_LP_NUMBER; PAD_COUNT];
            $( numbers[crate::peripherals::$gpio::NUMBER as usize] = $lp; )*
            numbers
        };
    };
}

/// The number of low-power pads. Every one of them can be armed at the same time.
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
    gpio: u8,
    /// The number that the low-power registers use for the pad.
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

/// Returns whether this pad ended the last sleep.
pub(crate) fn caused_wakeup(pin: &AnyPin<'_>) -> bool {
    WOKEN_PADS.contains(pin.number())
}

/// Copies the pads that ended the sleep from the path statuses into [`WOKEN_PADS`].
///
/// A sleep ends in one of two places: at the end of a light sleep, and in the sleep initialization
/// of a boot after a deep sleep. Both places call this function, and they call it after every
/// sleep, so that the record does not describe an earlier sleep. The boot calls it before it
/// releases the pads that the previous run armed, because that release also clears the `ext1`
/// selection.
#[crate::ram]
pub(crate) fn record_wakeup() {
    let cause = crate::rtc_cntl::wakeup_cause();

    for bank in GpioBank::ALL {
        // The digital path reports only through the interrupt of the pin. The interrupt status is
        // still set here, unless an interrupt handler ran first. A handler can only run first if
        // interrupts were enabled during the light sleep.
        let digital = if cause.contains(WakeupSource::Gpio) {
            bank.read_interrupt_status() & bank.listening().load(Ordering::Relaxed)
        } else {
            0
        };

        WOKEN_PADS.word(bank).store(digital, Ordering::Relaxed);
    }

    for (gpio, &lp) in LP_NUMBERS.iter().enumerate() {
        if lp != NO_LP_NUMBER && path::caused_wakeup(gpio as u8, cause) {
            WOKEN_PADS.set(gpio as u8, true);
        }
    }
}

/// Records that a pad can wake the chip, and registers the hooks that allocate the paths.
///
/// The mask bit alone does not wake the chip. A wake also needs the per-pin digital bit that
/// `listen` writes, or a low-power path that the entry hook assigns. If no pin can wake the chip,
/// the entry hook clears the mask bit again. A sleep with no other wakeup source is then refused,
/// instead of never ending.
pub(crate) fn enable() {
    WakeupSource::Gpio.enable_with_hooks(Some(entry_hook), Some(exit_hook));
}

fn disable() {
    WakeupSource::Gpio.disable();

    // Without this, a bit that a path of this chip still holds outlives the hook that clears it,
    // and the chip keeps a wakeup source that nobody requested.
    path::disable();
}

/// Returns the number that the low-power registers use for `gpio`, or `None` if they do not reach
/// the pad.
fn lp_number(gpio: u8) -> Option<u8> {
    match LP_NUMBERS[gpio as usize] {
        NO_LP_NUMBER => None,
        lp => Some(lp),
    }
}

/// Returns every pad that the low-power registers reach, as a pin number and a low-power number.
fn low_power_pads() -> impl Iterator<Item = (u8, u8)> {
    LP_NUMBERS
        .iter()
        .copied()
        .enumerate()
        .filter(|&(_, lp)| lp != NO_LP_NUMBER)
        .map(|(gpio, lp)| (gpio as u8, lp))
}

/// Returns the low-power number of every pad that the low-power registers reach.
#[cfg(not(sleep_ext1_version = "3"))]
fn low_power_numbers() -> impl Iterator<Item = u8> {
    low_power_pads().map(|(_, lp)| lp)
}

/// Returns whether a wakeup source that configures pads is enabled.
///
/// Call this before the initialization clears the mask. The result tells the initialization whether
/// it must release the pads that the previous run armed. ESP-IDF uses the same condition for
/// `esp_deep_sleep_wakeup_io_reset`.
pub(crate) fn wake_enabled() -> bool {
    let sources = crate::rtc_cntl::sleep::enabled_sources();

    #[allow(unused_mut)]
    let mut enabled = sources.contains(WakeupSource::Gpio);

    #[cfg(sleep_has_wakeup_source_ext0)]
    {
        enabled |= sources.contains(WakeupSource::Ext0);
    }
    #[cfg(sleep_has_wakeup_source_ext1)]
    {
        enabled |= sources.contains(WakeupSource::Ext1);
    }

    enabled
}

/// Releases the pads that the previous run armed for a deep sleep.
///
/// A deep sleep holds its wake pads, because it powers down the circuit that drives them. The wake
/// resets the chip, but it does not release the hold: the hold register is in the always-on domain.
/// A held pad would ignore its driver for the rest of the program, so the boot releases it.
///
/// The function releases the hold, and changes nothing else. A change of a pad function can take a
/// pad from a low-power core, which keeps its pads while it runs.
pub(crate) fn wake_io_reset() {
    path::wake_io_reset();
}

/// Records that sleep entry took the hold of a pad, so that the exit hook releases it.
#[cfg(not(sleep_ext1_version_is_set))]
fn hold_taken(gpio: u8) {
    PREPARED_PADS.set(gpio, true);
}

/// Assigns the listening pins to the hardware paths, and requests the power domains that they need.
#[crate::ram]
fn entry_hook(config: &mut WrappedSleepConfig<'_>) {
    let mut buffer = [Armed::NONE; MAX_ARMED];
    let (armed, mut digital) = collect(&mut buffer);

    // Deep sleep powers the high-performance GPIO peripheral down. The digital path then cannot
    // wake the chip, whatever number of pins listen.
    digital &= !config.is_deep_sleep();

    if armed.is_empty() && !digital {
        // No pad can wake the chip, so the record must go. A mask that still claims a GPIO wakeup
        // source lets a sleep start that no other source can end.
        disable();
        return;
    }

    if digital {
        // The digital path reads the pad through the high-performance GPIO peripheral.
        config.keep_alive(SleepResource::HpPeripherals);
    }

    path::allocate(armed, config);
}

/// Returns the pads to the digital GPIO peripheral after a sleep that did not reset the chip.
///
/// A light sleep is such a sleep, and so is a deep sleep that the hardware rejects. The hooks of a
/// rejected deep sleep run in the same place, because sleep entry prepared the same pads.
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

            low_level::pad_hold(lp, Some(false));

            // Only a chip with a low-power IO MUX moved the pad, so only such a chip has to move it
            // back. esp32c2 and esp32c3 keep the pad on the digital IO MUX.
            #[cfg(sleep_ext1_version_is_set)]
            low_level::set_config(lp, true, false, LpFunction::LP_GPIO);
        }
    }
}

/// Collects the listening pins that requested a low-power path, and returns whether any pin
/// listens.
///
/// Every listening pin can end a light sleep, so the record of the listening pins gives both
/// results. Neither result needs a read of the pin registers.
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

            // Only a pad that the low-power registers reach enters the mask, so it has a number.
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

/// Returns the level that wakes the chip through this pad, or `None` if the trigger sets no level.
///
/// The wake paths accept a level only, so an edge trigger becomes the level at the end of the edge.
/// This keeps the request of the user. `AnyEdge` ends at the level that the pin is not at now,
/// which is the meaning of "wake when the pin changes". This read of the pin can race with the pin,
/// but the result is safe. If the pin changes before the sleep starts, the path arms a level that
/// is already present, and the sleep is rejected or ends immediately.
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

/// Moves the pad to the low-power IO MUX, which makes it readable while the high-performance GPIO
/// peripheral is powered down.
///
/// Before a deep sleep this function also holds the pad, because deep sleep powers down the circuit
/// that drives it. A light sleep does not hold the pad. A light sleep powers down no circuit that
/// drives the pad, and a hold would freeze an output.
#[cfg(sleep_ext1_version_is_set)]
fn prepare_pad(pin: &Armed, deep: bool) {
    // The low-power IO MUX has its own pull resistors, and the digital ones stop to work as soon as
    // the low-power IO MUX takes the pad. Copy the resistors, or an undriven pad floats away from
    // the level that the user selected, and wakes the chip immediately.
    #[cfg(not(esp32h2))]
    {
        let digital = crate::gpio::io_mux_reg(pin.gpio).read();
        low_level::pullup_enable(pin.lp, digital.fun_wpu().bit());
        low_level::pulldown_enable(pin.lp, digital.fun_wpd().bit());
    }

    // esp32h2 reaches the pad through the digital IO MUX, so it has no low-power MUX to select.
    low_level::set_config(pin.lp, true, !cfg!(esp32h2), LpFunction::LP_GPIO);
    low_level::pad_hold(pin.lp, Some(deep));

    PREPARED_PADS.set(pin.gpio, true);
}

/// Disconnects the pads that no hold keeps, so that a powered-down output driver does not increase
/// the deep-sleep current.
///
/// No wakeup source controls this step, so every deep sleep does it, like
/// `esp_sleep_isolate_digital_gpio` in ESP-IDF. The step is destructive, because a pad loses the
/// peripheral function that drove it. It therefore runs at sleep entry, on the chips that cannot
/// hold a single pad through a deep sleep, and after the wakeup sources take the holds that they
/// need.
#[cfg(sleep_deep_sleep_needs_gpio_isolation)]
pub(crate) fn isolate_pads_for_deep_sleep() {
    use crate::{
        gpio::{AlternateFunction, OutputSignal, io_mux_reg},
        peripherals::LPWR,
    };

    // If the hold is disabled, no pad keeps its level through the sleep. ESP-IDF also skips the
    // isolation in that case.
    let dig_iso = LPWR::regs().dig_iso().read();
    let hold_enabled = !dig_iso.dg_pad_force_unhold().bit() && dig_iso.dg_pad_autohold_en().bit();
    if !hold_enabled {
        return;
    }
    for gpio in 0..PAD_COUNT as u8 {
        // Only the pads that the digital supply feeds leak current, and those are the pads with no
        // low-power number. The low-power pads have their own supply, which stays powered, and a
        // wake pad is always a low-power pad.
        if lp_number(gpio).is_some() {
            continue;
        }

        // A pad that the user holds keeps its level, which is the purpose of the hold. Isolation
        // would cancel that request.
        if crate::gpio::lp_io::is_digital_pad_held(gpio) {
            continue;
        }

        GPIO::regs()
            .func_out_sel_cfg(gpio as usize)
            .modify(|_, w| unsafe { w.bits(OutputSignal::GPIO as u32) });

        io_mux_reg(gpio).modify(|_, w| unsafe {
            w.fun_wpu().clear_bit();
            w.fun_wpd().clear_bit();
            w.fun_ie().clear_bit();
            // A pad that keeps a peripheral function increases the deep-sleep current.
            w.mcu_sel().bits(AlternateFunction::GPIO as u8)
        });
    }
}

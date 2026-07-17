#![cfg_attr(docsrs, procmacros::doc_replace(
    "lp_io" => {
        cfg(esp32) => "GPIO12",
        cfg(any(esp32s2, esp32s3)) => "GPIO21",
        cfg(esp32h2) => "GPIO8",
        _ => "GPIO1"
    }
))]
//! Low Power IO (LP_IO)
//!
//! # Overview
//!
//! The hardware provides GPIO pins with low-power capabilities and analog
//! functions. Depending on the device, these pins are controlled by an RTC IO
//! or LP IO peripheral.
//!
//! ## Configuration
//!
//! Low-power pins bypass the regular IO MUX and GPIO matrix so they can be used
//! by the ULP/LP core and low-power peripherals. They can remain operational
//! during deep sleep and can be used as wake-up sources.
//!
//! ## Examples
//!
//! ### Configure a Low-Power Pin as Output
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::gpio::lp_io::LowPowerOutput;
//! let lp_pin = LowPowerOutput::new(peripherals.__lp_io__);
//! # {after_snippet}
//! ```

use core::marker::PhantomData;

use super::{InputPin, OutputPin, RtcPin};

#[cfg_attr(lp_io_version = "esp32", path = "low_level/esp32.rs")]
#[cfg_attr(lp_io_version = "v2", path = "low_level/v2.rs")]
#[cfg_attr(lp_io_version = "v3", path = "low_level/v3.rs")]
#[cfg_attr(lp_io_version = "esp32c6", path = "low_level/esp32c6.rs")]
#[cfg_attr(lp_io_version = "esp32h2", path = "low_level/esp32h2.rs")]
#[cfg_attr(lp_io_version = "esp32p4", path = "low_level/esp32p4.rs")]
mod low_level;

// FIXME: RtcPin is necessary only as long as the sleep API takes &dyn RtcPin(WithResistors).
// After that has been resolved, we can simplify this hierarchy to only have LowPowerPin.

/// Trait implemented by pins with a known low-power pin number.
#[doc(hidden)]
pub trait LowPowerPin<const PIN: u8>: RtcPin {}

for_each_lp_function! {
    (($_signal:ident, RTC_GPIOn, $pin:literal), $gpio:ident) => {
        impl LowPowerPin<$pin> for crate::peripherals::$gpio<'_> {}
    };
    (($_signal:ident, LP_GPIOn, $pin:literal), $gpio:ident) => {
        impl LowPowerPin<$pin> for crate::peripherals::$gpio<'_> {}
    };
}

/// A GPIO output pin configured for low-power operation.
pub struct LowPowerOutput<'d, const PIN: u8> {
    phantom: PhantomData<&'d mut ()>,
}

impl<'d, const PIN: u8> LowPowerOutput<'d, PIN> {
    /// Creates a new output pin for use by the low-power core.
    #[instability::unstable]
    pub fn new<P>(pin: P) -> Self
    where
        P: LowPowerPin<PIN> + OutputPin + 'd,
    {
        let pin = low_level::init_pin(&pin, false);
        low_level::output_enable(pin, true);

        Self {
            phantom: PhantomData,
        }
    }
}

/// A GPIO input pin configured for low-power operation.
pub struct LowPowerInput<'d, const PIN: u8> {
    phantom: PhantomData<&'d mut ()>,
}

impl<'d, const PIN: u8> LowPowerInput<'d, PIN> {
    /// Creates a new input pin for use by the low-power core.
    #[instability::unstable]
    pub fn new<P>(pin: P) -> Self
    where
        P: LowPowerPin<PIN> + InputPin + 'd,
    {
        let pin = low_level::init_pin(&pin, true);
        low_level::input_enable(pin, true);
        low_level::pullup_enable(pin, false);
        low_level::pulldown_enable(pin, false);

        Self {
            phantom: PhantomData,
        }
    }

    /// Enables or disables the internal pull-up resistor.
    pub fn pullup_enable(&self, enable: bool) {
        low_level::pullup_enable(PIN, enable);
    }

    /// Enables or disables the internal pull-down resistor.
    pub fn pulldown_enable(&self, enable: bool) {
        low_level::pulldown_enable(PIN, enable);
    }
}

/// A GPIO open-drain output pin configured for low-power operation.
pub struct LowPowerOutputOpenDrain<'d, const PIN: u8> {
    phantom: PhantomData<&'d mut ()>,
}

impl<'d, const PIN: u8> LowPowerOutputOpenDrain<'d, PIN> {
    /// Creates a new open-drain output pin for use by the low-power core.
    #[instability::unstable]
    pub fn new<P>(pin: P) -> Self
    where
        P: LowPowerPin<PIN> + InputPin + OutputPin + 'd,
    {
        let gpio = pin.number();
        let pin = low_level::init_pin(&pin, true);
        low_level::set_open_drain_output(gpio, true);
        low_level::input_enable(pin, true);
        low_level::pullup_enable(pin, true);
        low_level::pulldown_enable(pin, false);
        low_level::output_enable(pin, true);

        Self {
            phantom: PhantomData,
        }
    }

    /// Enables or disables the internal pull-up resistor.
    pub fn pullup_enable(&self, enable: bool) {
        low_level::pullup_enable(PIN, enable);
    }

    /// Enables or disables the internal pull-down resistor.
    pub fn pulldown_enable(&self, enable: bool) {
        low_level::pulldown_enable(PIN, enable);
    }
}

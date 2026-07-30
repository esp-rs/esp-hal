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

define_lp_io_signals!();

#[cfg_attr(lp_io_version = "esp32", path = "low_level/esp32.rs")]
#[cfg_attr(lp_io_version = "v2", path = "low_level/v2.rs")]
#[cfg_attr(lp_io_version = "v3", path = "low_level/v3.rs")]
#[cfg_attr(lp_io_version = "esp32h2", path = "low_level/esp32h2.rs")]
#[cfg_attr(lp_io_version = "esp32p4", path = "low_level/esp32p4.rs")]
#[cfg_attr(lp_io_version = "v4", path = "low_level/v4.rs")]
mod low_level;

// FIXME: RtcPin is necessary only as long as the sleep API takes &dyn RtcPin(WithResistors).
// After that has been resolved, we can simplify this hierarchy to only have LowPowerPin.

/// Trait implemented by pins with a known low-power pin number.
#[doc(hidden)]
pub trait LowPowerPin<const PIN: u8>: RtcPin {}

for_each_lp_function! {
    (($_signal:ident, RTC_GPIOn, $pin:literal), $gpio:ident, $_af:literal, $_lp_in:tt $_lp_out:tt) => {
        impl LowPowerPin<$pin> for crate::peripherals::$gpio<'_> {}
    };
    (($_signal:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:literal, $_lp_in:tt $_lp_out:tt) => {
        impl LowPowerPin<$pin> for crate::peripherals::$gpio<'_> {}
    };
}

/// Configures an LP pin as an open-drain output with its pull-up enabled, and routes an LP
/// peripheral's signal pair to it through the LP GPIO matrix.
#[cfg(lp_io_has_gpio_matrix)]
#[cfg_attr(not(soc_has_lp_i2c0), expect(dead_code))]
pub(crate) fn connect_open_drain_signals(
    pin: &(impl RtcPin + InputPin + OutputPin),
    input: LpInputSignal,
    output: LpOutputSignal,
) {
    let lp_pin = low_level::init_pin(pin, true);
    low_level::set_open_drain_output(pin.number(), true);
    low_level::input_enable(lp_pin, true);
    low_level::pullup_enable(lp_pin, true);
    low_level::pulldown_enable(lp_pin, false);
    low_level::output_enable(lp_pin, true);

    route_input(lp_pin, input, true);
    route_output(lp_pin, output);
}

/// Configures an LP pin as an input and routes an LP peripheral's input signal to it.
#[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
pub(crate) fn connect_input_signal(pin: &(impl RtcPin + InputPin), input: LpInputSignal) {
    let mux_af = pin
        .lp_input_signals()
        .iter()
        .find(|(_, signal)| *signal == input)
        .map(|(af, _)| *af);

    let lp_pin = match mux_af {
        Some(af) => {
            let lp_pin = pin.rtc_number();
            low_level::set_pad_function(lp_pin, true, true, af);
            lp_pin
        }
        None => low_level::init_pin(pin, true),
    };

    low_level::input_enable(lp_pin, true);
    route_input(lp_pin, input, mux_af.is_none());
}

/// Configures an LP pin as an output and routes an LP peripheral's output signal to it.
#[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
pub(crate) fn connect_output_signal(pin: &(impl RtcPin + OutputPin), output: LpOutputSignal) {
    let mux_af = pin
        .lp_output_signals()
        .iter()
        .find(|(_, signal)| *signal == output)
        .map(|(af, _)| *af);

    match mux_af {
        Some(af) => {
            let lp_pin = pin.rtc_number();
            low_level::set_pad_function(lp_pin, false, true, af);
        }
        None => {
            let lp_pin = low_level::init_pin(pin, false);
            route_output(lp_pin, output);
        }
    }
}

/// Points an LP peripheral's input signal at `lp_pin`, either through the LP GPIO matrix or, when
/// the pad's own LP IO MUX function feeds the peripheral, straight from the pad.
#[cfg(lp_io_has_gpio_matrix)]
fn route_input(lp_pin: u8, input: LpInputSignal, use_gpio_matrix: bool) {
    crate::peripherals::LP_GPIO::regs()
        .func_in_sel_cfg(input as usize)
        .write(|w| unsafe {
            w.sig_in_sel().bit(use_gpio_matrix);
            w.in_inv_sel().clear_bit();
            w.in_sel().bits(lp_pin)
        });
}

#[cfg(lp_io_has_gpio_matrix)]
fn route_output(lp_pin: u8, output: LpOutputSignal) {
    crate::peripherals::LP_GPIO::regs()
        .func_out_sel_cfg(lp_pin as usize)
        .write(|w| unsafe {
            w.out_sel().bits(output as _);
            w.out_inv_sel().clear_bit();
            // Let the peripheral drive the output enable signal.
            w.oe_sel().clear_bit();
            w.oe_inv_sel().clear_bit()
        });
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

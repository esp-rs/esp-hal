#![cfg_attr(docsrs, procmacros::doc_replace(
    "lp_io" => {
        cfg(esp32) => "GPIO12",
        cfg(any(esp32s2, esp32s3)) => "GPIO21",
        cfg(esp32h2) => "GPIO8",
        _ => "GPIO1"
    },
    "lp_num" => {
        cfg(any(esp32s2, esp32s3)) => "21",
        _ => "1"
    }
))]
//! Low Power IO (LP_IO)
//!
//! # Overview
//!
//! The hardware provides GPIO pins with low-power capabilities and analog
//! functions. These pins are controlled by the LP IO peripheral, which some
//! devices call RTC IO.
//!
//! ## Configuration
//!
//! Low-power pins bypass the regular IO MUX and GPIO matrix so they can be used
//! by the ULP/LP core and low-power peripherals. They can remain operational
//! during deep sleep and can be used as wake-up sources.
#![cfg_attr(
    ulp_riscv_driver_supported,
    doc = "

## Handing off a pin to the low-power core

[`LowPowerInput`], [`LowPowerOutput`], and [`LowPowerOutputOpenDrain`] are opaque tokens that
represent a pad owned by the low-power domain. Obtain them by consuming a
[`Input`](crate::gpio::Input) or [`Output`](crate::gpio::Output) driver with
[`Input::into_lp`](crate::gpio::Input::into_lp), [`Output::into_lp`](crate::gpio::Output::into_lp),
or [`Output::into_open_drain_lp`](crate::gpio::Output::into_open_drain_lp). The conversion muxes the
pad into the low-power domain; afterwards the HP core can no longer drive it.

The `PIN` parameter of the tokens is the low-power pin number, which is how low-power core firmware
refers to the pin. The conversions verify at runtime that the pad they are given is that pin, and
hand the driver back when it is not.

The tokens can also be created straight from a pin with `LowPowerInput::new`,
`LowPowerOutput::new`, and `LowPowerOutputOpenDrain::new`, which check the pin number at compile
time but leave the pad's configuration up to the token.

## Examples

### Hand a pin to the LP core

```rust, no_run
# {before_snippet}
use esp_hal::gpio::{Level, Output, OutputConfig};
let pin = Output::new(peripherals.__lp_io__, Level::Low, OutputConfig::default());
let lp_pin = pin.into_lp::<__lp_num__>().unwrap();
# {after_snippet}
```
"
)]

#[cfg(ulp_riscv_driver_supported)]
use core::marker::PhantomData;

#[cfg(any(ulp_riscv_driver_supported, lp_io_has_gpio_matrix))]
use super::{InputPin, LpPin, OutputPin};

define_lp_io_signals!();
define_lp_functions!();

#[cfg_attr(lp_io_version = "esp32", path = "low_level/esp32.rs")]
#[cfg_attr(lp_io_version = "v2", path = "low_level/v2.rs")]
#[cfg_attr(lp_io_version = "v3", path = "low_level/v3.rs")]
#[cfg_attr(lp_io_version = "esp32h2", path = "low_level/esp32h2.rs")]
#[cfg_attr(lp_io_version = "esp32p4", path = "low_level/esp32p4.rs")]
#[cfg_attr(lp_io_version = "v4", path = "low_level/v4.rs")]
pub(crate) mod low_level;

/// Returns the trigger type that the low-power wakeup register of a pad needs to wake the chip at
/// `level`.
///
/// The field takes the same values as the digital interrupt type.
// Only the chips that arm the pads one by one have such a register. The other chips reach the pads
// through `ext1`, which takes a mask.
#[cfg(all(
    sleep_pin_wakeup_version_is_set,
    any(sleep_ext1_version = "1", not(sleep_ext1_version_is_set))
))]
pub(crate) const fn wake_trigger(level: super::Level) -> u8 {
    match level {
        super::Level::Low => super::Event::LowLevel as u8,
        super::Level::High => super::Event::HighLevel as u8,
    }
}

/// Trait implemented by pins with a known low-power pin number.
#[cfg(ulp_riscv_driver_supported)]
#[doc(hidden)]
pub trait LowPowerPin<const PIN: u8>: LpPin {}

#[cfg(ulp_riscv_driver_supported)]
for_each_lp_function! {
    (($_signal:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
        impl LowPowerPin<$pin> for crate::peripherals::$gpio<'_> {}
    };
}

/// Configures an LP pin as an open-drain output with its pull-up enabled, and routes an LP
/// peripheral's signal pair to it through the LP GPIO matrix.
#[cfg(lp_io_has_gpio_matrix)]
#[cfg_attr(not(soc_has_lp_i2c0), expect(dead_code))]
pub(crate) fn connect_open_drain_signals(
    pin: &(impl LpPin + InputPin + OutputPin),
    input: LpInputSignal,
    output: LpOutputSignal,
) {
    let lp_pin = low_level::init_pin(pin.lp_number(), true);
    low_level::set_open_drain_output(pin.number(), true);
    low_level::input_enable(lp_pin, true);
    low_level::pullup_enable(lp_pin, true);
    low_level::pulldown_enable(lp_pin, false);
    low_level::output_enable(lp_pin, true);

    route_input(lp_pin, input, true);
    route_output(lp_pin, output);
}

/// Returns an LP pin to its reset state, handing the pad back to the digital IO MUX.
///
/// A pin in this state no longer drives, senses or pulls anything, so any LP peripheral signal
/// still routed to it through the LP GPIO matrix cannot reach the pad.
#[cfg(lp_i2c_master_driver_supported)]
pub(crate) use low_level::reset_pin;

/// Configures an LP pin as an input and routes an LP peripheral's input signal to it.
#[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
pub(crate) fn connect_input_signal(pin: &(impl LpPin + InputPin), input: LpInputSignal) {
    let lp_pin = pin.lp_number();

    let mux_af = low_level::input_signals(lp_pin)
        .iter()
        .find(|(_, signal)| *signal == input)
        .map(|(af, _)| *af);

    match mux_af {
        Some(af) => low_level::set_config(lp_pin, true, true, af),
        None => {
            low_level::init_pin(lp_pin, true);
        }
    }

    low_level::input_enable(lp_pin, true);
    route_input(lp_pin, input, mux_af.is_none());
}

/// Configures an LP pin as an output and routes an LP peripheral's output signal to it.
#[cfg(all(lp_io_has_gpio_matrix, lp_uart_driver_supported))]
pub(crate) fn connect_output_signal(pin: &(impl LpPin + OutputPin), output: LpOutputSignal) {
    let lp_pin = pin.lp_number();

    let mux_af = low_level::output_signals(lp_pin)
        .iter()
        .find(|(_, signal)| *signal == output)
        .map(|(af, _)| *af);

    match mux_af {
        Some(af) => low_level::set_config(lp_pin, false, true, af),
        None => {
            low_level::init_pin(lp_pin, false);
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

/// Tokens to hand out a pin to a low-power CPU.
// FIXME: tokens should be 'static to be handed out.
#[cfg(ulp_riscv_driver_supported)]
mod ulp_tokens {
    use super::*;
    use crate::gpio::Pin;

    // Needed because AnyPin::lp_number is infallible
    fn lp_number(gpio: u8) -> Option<u8> {
        for_each_lp_function! {
            (($_signal:ident, LP_GPIOn, $pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
                if gpio == crate::peripherals::$gpio::NUMBER {
                    return Some($pin);
                }
            };
        }

        None
    }

    impl<'d> crate::gpio::Input<'d> {
        /// Hands the pin over to the low-power core.
        ///
        /// `PIN` is the low-power pin number, which is how low-power core firmware refers to the
        /// pin.
        ///
        /// # Errors
        ///
        /// Returns the driver unchanged if the pad is not the low-power pin numbered `PIN`.
        #[instability::unstable]
        pub fn into_lp<const PIN: u8>(self) -> Result<LowPowerInput<'d, PIN>, Self> {
            if lp_number(self.pin.pin.number()) != Some(PIN) {
                return Err(self);
            }

            Ok(LowPowerInput::new_untyped(self.pin.pin))
        }
    }

    impl<'d> crate::gpio::Output<'d> {
        /// Hands the pin over to the low-power core.
        ///
        /// `PIN` is the low-power pin number, which is how low-power core firmware refers to the
        /// pin.
        ///
        /// # Errors
        ///
        /// Returns the driver unchanged if the pad is not the low-power pin numbered `PIN`.
        #[instability::unstable]
        pub fn into_lp<const PIN: u8>(self) -> Result<LowPowerOutput<'d, PIN>, Self> {
            if lp_number(self.pin.pin.number()) != Some(PIN) {
                return Err(self);
            }

            Ok(LowPowerOutput::new_untyped(self.pin.pin))
        }

        /// Hands the pin over to the low-power core as an open-drain output.
        ///
        /// `PIN` is the low-power pin number, which is how low-power core firmware refers to the
        /// pin. The pad's pull-up is enabled, regardless of the driver's configuration.
        ///
        /// # Errors
        ///
        /// Returns the driver unchanged if the pad is not the low-power pin numbered `PIN`.
        #[instability::unstable]
        pub fn into_open_drain_lp<const PIN: u8>(
            self,
        ) -> Result<LowPowerOutputOpenDrain<'d, PIN>, Self> {
            if lp_number(self.pin.pin.number()) != Some(PIN) {
                return Err(self);
            }

            Ok(LowPowerOutputOpenDrain::new_untyped(self.pin.pin))
        }
    }

    /// A GPIO output pin configured for low-power operation.
    #[instability::unstable]
    pub struct LowPowerOutput<'d, const PIN: u8> {
        pub(super) phantom: PhantomData<&'d mut ()>,
    }

    impl<'d, const PIN: u8> LowPowerOutput<'d, PIN> {
        /// Creates a new output pin for use by the low-power core.
        #[instability::unstable]
        pub fn new<P>(pin: P) -> Self
        where
            P: LowPowerPin<PIN> + OutputPin + 'd,
        {
            Self::new_untyped(pin)
        }

        /// Takes a pad that the caller confirmed to be the low-power pin with the number `PIN`.
        pub(super) fn new_untyped<P>(_pin: P) -> Self
        where
            P: Pin + 'd,
        {
            low_level::init_pin(PIN, false);
            low_level::output_enable(PIN, true);

            Self {
                phantom: PhantomData,
            }
        }
    }

    /// A GPIO input pin configured for low-power operation.
    #[instability::unstable]
    pub struct LowPowerInput<'d, const PIN: u8> {
        pub(super) phantom: PhantomData<&'d mut ()>,
    }

    impl<'d, const PIN: u8> LowPowerInput<'d, PIN> {
        /// Creates a new input pin for use by the low-power core.
        #[instability::unstable]
        pub fn new<P>(pin: P) -> Self
        where
            P: LowPowerPin<PIN> + InputPin + 'd,
        {
            Self::new_untyped(pin)
        }

        /// Takes a pad that the caller confirmed to be the low-power pin with the number `PIN`.
        pub(super) fn new_untyped<P>(_pin: P) -> Self
        where
            P: Pin + 'd,
        {
            low_level::init_pin(PIN, true);
            low_level::input_enable(PIN, true);
            low_level::pullup_enable(PIN, false);
            low_level::pulldown_enable(PIN, false);

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
    #[instability::unstable]
    pub struct LowPowerOutputOpenDrain<'d, const PIN: u8> {
        pub(super) phantom: PhantomData<&'d mut ()>,
    }

    impl<'d, const PIN: u8> LowPowerOutputOpenDrain<'d, PIN> {
        /// Creates a new open-drain output pin for use by the low-power core.
        #[instability::unstable]
        pub fn new<P>(pin: P) -> Self
        where
            P: LowPowerPin<PIN> + InputPin + OutputPin + 'd,
        {
            Self::new_untyped(pin)
        }

        /// Takes a pad that the caller confirmed to be the low-power pin with the number `PIN`.
        pub(super) fn new_untyped<P>(pin: P) -> Self
        where
            P: Pin + 'd,
        {
            let gpio = pin.number();
            low_level::init_pin(PIN, true);
            low_level::set_open_drain_output(gpio, true);
            low_level::input_enable(PIN, true);
            low_level::pullup_enable(PIN, true);
            low_level::pulldown_enable(PIN, false);
            low_level::output_enable(PIN, true);

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
}

#[cfg(ulp_riscv_driver_supported)]
pub use ulp_tokens::*;

//! # General Purpose Input/Output
//!
//! ## Overview
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.
//!
//! This driver supports various operations on GPIO pins, primarily manipulating
//! the pin state (setting high/low, toggling).
//!
//! This module also implements a number of traits from `embedded-hal` to
//! provide a common interface for GPIO pins.
//!
//! ## Examples
//!
//! ```rust
//! fn main(gpio0: Input<0>, gpio1: Output<1>) -> ! {
//!     loop {
//!         let input_state: bool = gpio0.input_state();
//!         gpio.set_output(input_state);
//!
//!         esp_lp_hal::delay::Delay.delay_millis(50);
//!     }
//! }
//! ```

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c6")] {
        type LpIo = crate::pac::LP_IO;
        const MAX_GPIO_PIN: u8 = 7;
    } else {
        type LpIo = crate::pac::RTC_IO;
        const MAX_GPIO_PIN: u8 = 21;
    }
}

/// GPIO input driver
pub struct Input<const PIN: u8>;

impl<const PIN: u8> Input<PIN> {
    /// Read the input state/level of the pin.
    pub fn input_state(&self) -> bool {
        unsafe { &*LpIo::PTR }.in_().read().bits() >> PIN & 0x1 != 0
    }
}

/// GPIO output driver
pub struct Output<const PIN: u8>;

impl<const PIN: u8> Output<PIN> {
    /// Read the output state/level of the pin.
    pub fn output_state(&self) -> bool {
        unsafe { &*LpIo::PTR }.out().read().bits() >> PIN & 0x1 != 0
    }

    /// Set the output state/level of the pin.
    pub fn set_output(&mut self, on: bool) {
        if on {
            unsafe { &*LpIo::PTR }
                .out_w1ts()
                .write(|w| unsafe { w.out_data_w1ts().bits(1 << PIN) });
        } else {
            unsafe { &*LpIo::PTR }
                .out_w1tc()
                .write(|w| unsafe { w.out_data_w1tc().bits(1 << PIN) });
        }
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_output<const PIN: u8>() -> Option<Output<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Output)
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_input<const PIN: u8>() -> Option<Input<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Input)
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::ErrorType for Input<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::ErrorType for Output<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::InputPin for Input<PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::OutputPin for Output<PIN> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output(true);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::StatefulOutputPin for Output<PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

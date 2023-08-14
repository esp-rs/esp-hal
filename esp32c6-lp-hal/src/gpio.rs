//! Low-power GPIO driver
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.

use core::{convert::Infallible, marker::PhantomData};

use esp32c6_lp::LP_IO;

#[non_exhaustive]
pub struct IO {
    _peripheral: LP_IO,

    pub gpio0: GpioPin<0, Unknown>,
    pub gpio1: GpioPin<1, Unknown>,
    pub gpio2: GpioPin<2, Unknown>,
    pub gpio3: GpioPin<3, Unknown>,
    pub gpio4: GpioPin<4, Unknown>,
    pub gpio5: GpioPin<5, Unknown>,
    pub gpio6: GpioPin<6, Unknown>,
    pub gpio7: GpioPin<7, Unknown>,
}

impl IO {
    pub fn new(peripheral: LP_IO) -> Self {
        Self {
            _peripheral: peripheral,

            gpio0: GpioPin::new(),
            gpio1: GpioPin::new(),
            gpio2: GpioPin::new(),
            gpio3: GpioPin::new(),
            gpio4: GpioPin::new(),
            gpio5: GpioPin::new(),
            gpio6: GpioPin::new(),
            gpio7: GpioPin::new(),
        }
    }
}

pub struct Unknown {}

pub struct Input;

pub struct Output;

#[non_exhaustive]
pub struct GpioPin<const PIN: u8, MODE> {
    phantom: PhantomData<MODE>,
}

impl<const PIN: u8, MODE> GpioPin<PIN, MODE> {
    fn new() -> Self {
        GpioPin {
            phantom: PhantomData,
        }
    }

    /// Assuming the GPIO is already configured by the HP core this makes the
    /// GPIO into an input pin.
    pub fn into_input(self) -> GpioPin<PIN, Input> {
        GpioPin::new()
    }

    /// Assuming the GPIO is already configured by the HP core this makes the
    /// GPIO into an output pin.
    pub fn into_output(self) -> GpioPin<PIN, Output> {
        GpioPin::new()
    }
}

impl<const PIN: u8> GpioPin<PIN, Input> {
    fn input_state(&self) -> bool {
        unsafe { &*LP_IO::PTR }.in_.read().bits() >> PIN & 0x1 != 0
    }
}

impl<const PIN: u8> GpioPin<PIN, Output> {
    fn output_state(&self) -> bool {
        unsafe { &*LP_IO::PTR }.out.read().bits() >> PIN & 0x1 != 0
    }

    fn set_output_low(&mut self) {
        unsafe { &*LP_IO::PTR }
            .out_w1tc
            .write(|w| w.out_data_w1tc().variant(1 << PIN));
    }

    fn set_output_high(&mut self) {
        unsafe { &*LP_IO::PTR }
            .out_w1ts
            .write(|w| w.out_data_w1ts().variant(1 << PIN));
    }
}

impl<const PIN: u8> embedded_hal::digital::v2::InputPin for GpioPin<PIN, Input> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

impl<const PIN: u8> embedded_hal::digital::v2::OutputPin for GpioPin<PIN, Output> {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output_high();
        Ok(())
    }
}

impl<const PIN: u8> embedded_hal::digital::v2::StatefulOutputPin for GpioPin<PIN, Output> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

impl<const PIN: u8> embedded_hal::digital::v2::toggleable::Default for GpioPin<PIN, Output> {}

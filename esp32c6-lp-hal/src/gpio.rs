//! LP-GPIO driver
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.

use core::{convert::Infallible, marker::PhantomData};

use embedded_hal::digital::v2::OutputPin;

pub struct Unknown {}

pub struct Output;

#[non_exhaustive]
pub struct GpioPin<const PIN: u8, MODE> {
    phantom: PhantomData<MODE>,
}

#[non_exhaustive]
pub struct Io {
    _peripheral: esp32c6_lp::LP_IO,

    pub gpio0: GpioPin<0, Unknown>,
    pub gpio1: GpioPin<1, Unknown>,
    pub gpio2: GpioPin<2, Unknown>,
    pub gpio3: GpioPin<3, Unknown>,
    pub gpio4: GpioPin<4, Unknown>,
    pub gpio5: GpioPin<5, Unknown>,
    pub gpio6: GpioPin<6, Unknown>,
    pub gpio7: GpioPin<7, Unknown>,
}

impl Io {
    pub fn new(peripheral: esp32c6_lp::LP_IO) -> Self {
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

impl<const PIN: u8, MODE> GpioPin<PIN, MODE> {
    fn new() -> Self {
        GpioPin {
            phantom: PhantomData::default(),
        }
    }

    /// Assuming the GPIO is already configured by the HP core this makes the
    /// GPIO into an output pin.
    pub fn into_output(self) -> GpioPin<PIN, Output> {
        GpioPin::new()
    }
}

impl<const PIN: u8> OutputPin for GpioPin<PIN, Output> {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        let lp_gpio = unsafe { &*esp32c6_lp::LP_IO::PTR };
        lp_gpio
            .out_w1tc
            .write(|w| w.out_data_w1tc().variant(1 << PIN));
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        let lp_gpio = unsafe { &*esp32c6_lp::LP_IO::PTR };
        lp_gpio
            .out_w1ts
            .write(|w| w.out_data_w1ts().variant(1 << PIN));
        Ok(())
    }
}

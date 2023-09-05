//! Low-power GPIO driver
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.

use core::{convert::Infallible, marker::PhantomData};

use esp32c6_lp::LP_IO;

#[doc(hidden)]
pub unsafe fn conjour<MODE, const PIN: u8>() -> Option<GpioPin<MODE, PIN>> {
    if PIN > 7 {
        None
    } else {
        Some(GpioPin {
            phantom: PhantomData,
        })
    }
}

pub struct Unknown {}

pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct Floating;

pub struct PullDown;

pub struct PullUp;

pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct PushPull;

#[non_exhaustive]
pub struct GpioPin<MODE, const PIN: u8> {
    phantom: PhantomData<MODE>,
}

impl<MODE, const PIN: u8> GpioPin<Input<MODE>, PIN> {
    fn input_state(&self) -> bool {
        unsafe { &*LP_IO::PTR }.in_.read().bits() >> PIN & 0x1 != 0
    }
}

impl<MODE, const PIN: u8> GpioPin<Output<MODE>, PIN> {
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

impl<MODE, const PIN: u8> embedded_hal::digital::v2::InputPin for GpioPin<Input<MODE>, PIN> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

impl<MODE, const PIN: u8> embedded_hal::digital::v2::OutputPin for GpioPin<Output<MODE>, PIN> {
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

impl<MODE, const PIN: u8> embedded_hal::digital::v2::StatefulOutputPin
    for GpioPin<Output<MODE>, PIN>
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

impl<MODE, const PIN: u8> embedded_hal::digital::v2::toggleable::Default
    for GpioPin<Output<MODE>, PIN>
{
}

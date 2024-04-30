//! Low-power GPIO driver
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.

use core::marker::PhantomData;

#[cfg(feature = "esp32c6")]
type LpIo = crate::pac::LP_IO;
#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
type LpIo = crate::pac::RTC_IO;

pub struct Unknown;

pub struct Input;

pub struct Output;

pub struct GpioPin<MODE, const PIN: u8> {
    phantom: PhantomData<MODE>,
}

impl<const PIN: u8> GpioPin<Input, PIN> {
    pub fn input_state(&self) -> bool {
        unsafe { &*LpIo::PTR }.in_().read().bits() >> PIN & 0x1 != 0
    }
}

impl<const PIN: u8> GpioPin<Output, PIN> {
    pub fn output_state(&self) -> bool {
        unsafe { &*LpIo::PTR }.out().read().bits() >> PIN & 0x1 != 0
    }

    pub fn set_output(&mut self, on: bool) {
        if on {
            unsafe { &*LpIo::PTR }
                .out_w1ts()
                .write(|w| w.out_data_w1ts().variant(1 << PIN));
        } else {
            unsafe { &*LpIo::PTR }
                .out_w1tc()
                .write(|w| w.out_data_w1tc().variant(1 << PIN));
        }
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure<MODE, const PIN: u8>() -> Option<GpioPin<MODE, PIN>> {
    if PIN > 7 {
        None
    } else {
        Some(GpioPin {
            phantom: PhantomData,
        })
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::InputPin for GpioPin<Input, PIN> {
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::OutputPin for GpioPin<Output, PIN> {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output(true);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::StatefulOutputPin for GpioPin<Output, PIN> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::toggleable::Default for GpioPin<Output, PIN> {}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::ErrorType for GpioPin<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::InputPin for GpioPin<Input, PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::OutputPin for GpioPin<Output, PIN> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output(true);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::StatefulOutputPin for GpioPin<Output, PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

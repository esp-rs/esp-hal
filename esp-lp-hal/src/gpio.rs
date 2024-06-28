//! Low-power GPIO driver
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.

#[cfg(feature = "esp32c6")]
type LpIo = crate::pac::LP_IO;
#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
type LpIo = crate::pac::RTC_IO;

#[cfg(feature = "esp32c6")]
const MAX_GPIO_PIN: u8 = 7;
#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
const MAX_GPIO_PIN: u8 = 21;

#[non_exhaustive]
pub struct Input<const PIN: u8> {}

impl<const PIN: u8> Input<PIN> {
    pub fn input_state(&self) -> bool {
        unsafe { &*LpIo::PTR }.in_().read().bits() >> PIN & 0x1 != 0
    }
}

#[non_exhaustive]
pub struct Output<const PIN: u8> {}

impl<const PIN: u8> Output<PIN> {
    pub fn output_state(&self) -> bool {
        unsafe { &*LpIo::PTR }.out().read().bits() >> PIN & 0x1 != 0
    }

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
        Some(Output {})
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_input<const PIN: u8>() -> Option<Input<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Input {})
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::InputPin for Input<PIN> {
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::OutputPin for Output<PIN> {
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
impl<const PIN: u8> embedded_hal_02::digital::v2::StatefulOutputPin for Output<PIN> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<const PIN: u8> embedded_hal_02::digital::v2::toggleable::Default for Output<PIN> {}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::ErrorType for Input<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::ErrorType for Output<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::InputPin for Input<PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.input_state())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

#[cfg(feature = "embedded-hal-1")]
impl<const PIN: u8> embedded_hal_1::digital::OutputPin for Output<PIN> {
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
impl<const PIN: u8> embedded_hal_1::digital::StatefulOutputPin for Output<PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_state())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_high()?)
    }
}

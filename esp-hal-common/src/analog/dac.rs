//! # Analog peripherals - Digital to Analog Converter
//!
//! ## Overview
//! The `DAC` module is part of the `Analog` driver designed for ESP
//! microcontrollers, providing functionalities for `digital-to-analog`
//! conversion.
//!
//! This module simplifies digital-to-analog conversion on ESP microcontrollers,
//! enabling precise control over analog output signals. Developers can choose
//! the `DAC` channel they want to use based on the GPIO pin assignments for
//! each channel. By providing a unified interface for DAC control, the module
//! makes it easier for users to generate accurate analog voltages in their
//! applications, such as audio generation, sensor calibration, and analog
//! signal synthesis.
use crate::peripherals::{RTC_IO, SENS};
pub trait DAC {
    fn write(&mut self, value: u8);
}

trait DAC1Impl {
    fn set_power(self) -> Self
    where
        Self: Sized,
    {
        #[cfg(esp32s2)]
        {
            let sensors = unsafe { &*SENS::ptr() };
            sensors
                .sar_dac_ctrl1
                .modify(|_, w| w.dac_clkgate_en().set_bit());
        }

        let rtcio = unsafe { &*RTC_IO::ptr() };

        rtcio.pad_dac1.modify(|_, w| {
            w.pdac1_dac_xpd_force().set_bit();
            w.pdac1_xpd_dac().set_bit()
        });

        self
    }

    fn write(&mut self, value: u8) {
        let rtcio = unsafe { &*RTC_IO::ptr() };

        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_dac_ctrl2
            .modify(|_, w| w.dac_cw_en1().clear_bit());

        rtcio
            .pad_dac1
            .modify(|_, w| unsafe { w.pdac1_dac().bits(value) });
    }
}

trait DAC2Impl {
    fn set_power(self) -> Self
    where
        Self: Sized,
    {
        #[cfg(esp32s2)]
        {
            let sensors = unsafe { &*SENS::ptr() };
            sensors
                .sar_dac_ctrl1
                .modify(|_, w| w.dac_clkgate_en().set_bit());
        }

        let rtcio = unsafe { &*RTC_IO::ptr() };

        rtcio.pad_dac2.modify(|_, w| {
            w.pdac2_dac_xpd_force().set_bit();
            w.pdac2_xpd_dac().set_bit()
        });

        self
    }

    fn write(&mut self, value: u8) {
        let rtcio = unsafe { &*RTC_IO::ptr() };

        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_dac_ctrl2
            .modify(|_, w| w.dac_cw_en2().clear_bit());

        rtcio
            .pad_dac2
            .modify(|_, w| unsafe { w.pdac2_dac().bits(value) });
    }
}

macro_rules! impl_dac {
    ($($number:literal => $gpio:ident),+) => {
        $(
            paste::paste! {
                use $crate::analog::dac::[<DAC $number Impl>];

                #[doc = "DAC channel " $number]
                pub struct [<DAC $number>]<'d, DAC> {
                    _dac: $crate::peripheral::PeripheralRef<'d, DAC>,
                    _private: ::core::marker::PhantomData<()>,
                }

                impl<'d, DAC> [<DAC $number Impl>] for [<DAC $number>]<'d, DAC> {}

                impl<'d, DAC> [<DAC $number>]<'d, DAC> {
                    /// Constructs a new DAC instance
                    pub fn dac(
                        dac: impl $crate::peripheral::Peripheral<P = DAC> +'d,
                        _pin: $crate::gpio::$gpio<$crate::gpio::Analog>,
                    ) -> Result<Self, ()> {
                        let dac = Self {
                            _dac: dac.into_ref(),
                            _private: ::core::marker::PhantomData,
                        }
                        .set_power();
                        Ok(dac)
                    }

                    /// Writes the given value
                    ///
                    /// For each DAC channel, the output analog voltage can be calculated as follows:
                    /// DACn_OUT = VDD3P3_RTC * PDACn_DAC/256
                    pub fn write(&mut self, value: u8) {
                        [<DAC $number Impl>]::write(self, value)
                    }
                }
            }
        )+
    };
}

pub use implementation::*;

#[cfg(esp32)]
mod implementation {
    //! Digital to analog (DAC) conversion.
    //!
    //! This module provides functions for controlling two digital to
    //! analog converters, available on ESP32: `DAC1` and `DAC2`.
    //!
    //! The DAC1 is available on the GPIO pin 25, and DAC2 on pin 26.

    impl_dac!(1 => Gpio25, 2 => Gpio26);
}

#[cfg(esp32s2)]
mod implementation {
    //! Digital to analog (DAC) conversion.
    //!
    //! This module provides functions for controlling two digital to
    //! analog converters, available on ESP32-S2: `DAC1` and `DAC2`.
    //!
    //! The DAC1 is available on the GPIO pin 17, and DAC2 on pin 18.

    impl_dac!(1 => Gpio17, 2 => Gpio18);
}

//! Analog peripherals
//! 
//! ## Overview
//! The `Analog` Driver is a module designed for ESP microcontrollers, that provides an interface
//! to interact with analog peripherals on the chip. The module includes support
//! for `Analog-to-Digital Converters (ADC)` and `Digital-to-Analog Converters (DAC)`, offering
//! functionality for precise analog measurements and generating analog output signals.
//! 
//! The `ADC` functionality in the `analog` driver enables users to perform analog-to-digital conversions,
//! allowing them to measure real-world analog signals with high accuracy. The module provides access
//! to multiple ADC units, such as `ADC1` and `ADC2`, which may differ based on the specific
//! ESP microcontroller being used.
//! 
//! The `DAC` functionality in the analog driver enables users to generate analog output signals
//! with precise control over voltage levels. The module supports multiple DAC units,
//! such as `DAC1` and `DAC2`, which may vary depending on the specific ESP microcontroller.
//! 
//! #### Xtensa architecture
//! For ESP microcontrollers using the `Xtensa` architecture, the driver provides access
//! to the `SENS` peripheral, allowing users to split it into independent parts
//! using the [`SensExt`] trait. This extension trait provides access to the following analog peripherals:
//!   * ADC1
//!   * ADC2
//!   * DAC1
//!   * DAC2
//! 
//! #### RISC-V architecture
//! For ESP microcontrollers using the `RISC-V` architecture, the driver provides
//! access to the `APB_SARADC` peripheral. The `SarAdcExt` trait allows users to split this peripheral
//! into independent parts, providing access to the following analog peripheral:
//!   * ADC1
//!   * ADC2
//! 
//! ## Examples
//! #### ADC on Risc-V architecture
//! ```no_run
//! // Create ADC instances
//! let analog = peripherals.APB_SARADC.split();
//!
//! let mut adc1_config = AdcConfig::new();
//!
//! let mut pin = adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
//!
//! let mut adc1 = ADC::<ADC1>::adc(
//!     &mut system.peripheral_clock_control,
//!     analog.adc1,
//!     adc1_config,
//! )
//! .unwrap();
//!
//! let mut delay = Delay::new(&clocks);
//!
//! loop {
//!     let pin_value: u16 = nb::block!(adc1.read(&mut pin)).unwrap();
//!     println!("PIN2 ADC reading = {}", pin_value);
//!     delay.delay_ms(1500u32);
//! } 
//! ```
//! #### ADC on Xtensa architecture
//! ```no_run
//! // Create ADC instances
//! let analog = peripherals.SENS.split();
//!
//! let mut adc1_config = AdcConfig::new();
//!
//! let mut pin3 =
//!     adc1_config.enable_pin(io.pins.gpio3.into_analog(), Attenuation::Attenuation11dB);
//!
//! let mut adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();
//!
//! let mut delay = Delay::new(&clocks);
//!
//! loop {
//!     let pin3_value: u16 = nb::block!(adc1.read(&mut pin3)).unwrap();
//!     println!("PIN3 ADC reading = {}", pin3_value);
//!     delay.delay_ms(1500u32);
//! }
//! ```

#[cfg_attr(esp32, path = "adc/esp32.rs")]
#[cfg_attr(riscv, path = "adc/riscv.rs")]
#[cfg_attr(any(esp32s2, esp32s3), path = "adc/xtensa.rs")]
pub mod adc;
#[cfg(dac)]
pub mod dac;

/// A helper trait to do calibrated samples fitting
pub trait AdcCalScheme<ADCI>: Sized {
    /// Instantiate scheme
    fn new_cal(atten: adc::Attenuation) -> Self;

    /// Get ADC calibration value to set to ADC unit
    fn adc_cal(&self) -> u16 {
        0
    }

    /// Convert ADC value
    fn adc_val(&self, val: u16) -> u16 {
        val
    }
}

impl<ADCI> AdcCalScheme<ADCI> for () {
    fn new_cal(_atten: adc::Attenuation) -> Self {
        ()
    }
}

/// A helper trait to get access to ADC calibration efuses
pub trait AdcCalEfuse {
    /// Get ADC calibration init code
    ///
    /// Returns digital value for zero voltage for a given attenuation
    fn get_init_code(atten: adc::Attenuation) -> Option<u16>;

    /// Get ADC calibration reference point voltage
    ///
    /// Returns reference voltage (millivolts) for a given attenuation
    fn get_cal_mv(atten: adc::Attenuation) -> u16;

    /// Get ADC calibration reference point digital value
    ///
    /// Returns digital value for reference voltage for a given attenuation
    fn get_cal_code(atten: adc::Attenuation) -> Option<u16>;
}

pub struct ADC1 {
    _private: (),
}

pub struct ADC2 {
    _private: (),
}

pub struct DAC1 {
    _private: (),
}

pub struct DAC2 {
    _private: (),
}

impl core::ops::Deref for ADC1 {
    type Target = ADC1;

    fn deref(&self) -> &Self::Target {
        self
    }
}

impl core::ops::DerefMut for ADC1 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self
    }
}

impl crate::peripheral::Peripheral for ADC1 {
    type P = ADC1;
    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        ADC1 { _private: () }
    }
}

impl crate::peripheral::sealed::Sealed for ADC1 {}

impl crate::peripheral::Peripheral for ADC2 {
    type P = ADC2;
    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        ADC2 { _private: () }
    }
}

impl crate::peripheral::sealed::Sealed for ADC2 {}

impl crate::peripheral::Peripheral for DAC1 {
    type P = DAC1;
    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        DAC1 { _private: () }
    }
}

impl crate::peripheral::sealed::Sealed for DAC1 {}

impl crate::peripheral::Peripheral for DAC2 {
    type P = DAC2;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        DAC2 { _private: () }
    }
}

impl crate::peripheral::sealed::Sealed for DAC2 {}

cfg_if::cfg_if! {
    if #[cfg(xtensa)] {
        use crate::peripherals::SENS;

        pub struct AvailableAnalog {
            pub adc1: ADC1,
            pub adc2: ADC2,
            pub dac1: DAC1,
            pub dac2: DAC2,
        }

        /// Extension trait to split a SENS peripheral in independent parts
        pub trait SensExt {
            fn split(self) -> AvailableAnalog;
        }

        impl SensExt for SENS {
            fn split(self) -> AvailableAnalog {
                AvailableAnalog {
                    adc1: ADC1 {
                        _private: (),
                    },
                    adc2: ADC2 {
                        _private: (),
                    },
                    dac1: DAC1 {
                        _private: (),
                    },
                    dac2: DAC2 {
                        _private: (),
                    },
                }
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(riscv)] {
        use crate::peripherals::APB_SARADC;

        pub struct AvailableAnalog {
            pub adc1: ADC1,
            #[cfg(esp32c3)]
            pub adc2: ADC2,
        }

        /// Extension trait to split a APB_SARADC peripheral in independent parts
        pub trait SarAdcExt {
            fn split(self) -> AvailableAnalog;
        }

        impl<'d, T: crate::peripheral::Peripheral<P = APB_SARADC> + 'd> SarAdcExt for T {
            fn split(self) -> AvailableAnalog {
                AvailableAnalog {
                    adc1: ADC1 {
                        _private: (),
                    },
                    #[cfg(esp32c3)]
                    adc2: ADC2 {
                        _private: (),
                    },
                }
            }
        }
    }
}

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

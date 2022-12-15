#[cfg_attr(esp32, path = "adc/esp32.rs")]
#[cfg_attr(esp32c2, path = "adc/riscv.rs")]
#[cfg_attr(esp32c3, path = "adc/riscv.rs")]
#[cfg_attr(esp32s2, path = "adc/xtensa.rs")]
#[cfg_attr(esp32s3, path = "adc/xtensa.rs")]
pub mod adc;
#[cfg(dac)]
pub mod dac;

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

impl crate::peripheral::Peripheral for &mut ADC1 {
    type P = ADC1;
    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        ADC1 { _private: () }
    }
}

impl core::ops::Deref for ADC2 {
    type Target = ADC2;

    fn deref(&self) -> &Self::Target {
        self
    }
}

impl core::ops::DerefMut for ADC2 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self
    }
}

impl crate::peripheral::Peripheral for ADC2 {
    type P = ADC2;
    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        ADC2 { _private: () }
    }
}

impl crate::peripheral::Peripheral for &mut ADC2 {
    type P = ADC2;
    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        ADC2 { _private: () }
    }
}


cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2, esp32s3))] {

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
    if #[cfg(esp32c3)] {

        use crate::peripherals::APB_SARADC;

        pub struct AvailableAnalog {
            pub adc1: ADC1,
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
                    adc2: ADC2 {
                        _private: (),
                    },
                }
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {

        use crate::peripherals::APB_SARADC;

        pub struct AvailableAnalog {
            pub adc1: ADC1,
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
                }
            }
        }
    }
}

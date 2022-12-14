#[cfg_attr(esp32, path = "adc/esp32.rs")]
#[cfg_attr(esp32c2, path = "adc/riscv.rs")]
#[cfg_attr(esp32c3, path = "adc/riscv.rs")]
#[cfg_attr(esp32s2, path = "adc/xtensa.rs")]
#[cfg_attr(esp32s3, path = "adc/xtensa.rs")]
pub mod adc;
#[cfg(dac)]
pub mod dac;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2, esp32s3))] {
        use core::marker::PhantomData;

        use crate::peripherals::SENS;

        pub struct ADC1 {
            _private: PhantomData<()>,
        }
        pub struct ADC2 {
            _private: PhantomData<()>,
        }

        pub struct DAC1 {
            _private: PhantomData<()>,
        }

        pub struct DAC2 {
            _private: PhantomData<()>,
        }

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
                        _private: PhantomData,
                    },
                    adc2: ADC2 {
                        _private: PhantomData,
                    },
                    dac1: DAC1 {
                        _private: PhantomData,
                    },
                    dac2: DAC2 {
                        _private: PhantomData,
                    },
                }
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32c3)] {
        use core::marker::PhantomData;

        use crate::peripherals::APB_SARADC;

        pub struct ADC1 {
            _private: PhantomData<()>,
        }
        pub struct ADC2 {
            _private: PhantomData<()>,
        }

        pub struct AvailableAnalog {
            pub adc1: ADC1,
            pub adc2: ADC2,
        }

        /// Extension trait to split a APB_SARADC peripheral in independent parts
        pub trait SarAdcExt {
            fn split(self) -> AvailableAnalog;
        }

        impl SarAdcExt for APB_SARADC {
            fn split(self) -> AvailableAnalog {
                AvailableAnalog {
                    adc1: ADC1 {
                        _private: PhantomData,
                    },
                    adc2: ADC2 {
                        _private: PhantomData,
                    },
                }
            }
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32c2)] {
        use core::marker::PhantomData;

        use crate::peripherals::APB_SARADC;

        pub struct ADC1 {
            _private: PhantomData<()>,
        }

        pub struct AvailableAnalog {
            pub adc1: ADC1,
        }

        /// Extension trait to split a APB_SARADC peripheral in independent parts
        pub trait SarAdcExt {
            fn split(self) -> AvailableAnalog;
        }

        impl SarAdcExt for APB_SARADC {
            fn split(self) -> AvailableAnalog {
                AvailableAnalog {
                    adc1: ADC1 {
                        _private: PhantomData,
                    },
                }
            }
        }
    }
}

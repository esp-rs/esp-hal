#[cfg(not(any(feature = "esp32c3", feature = "esp32s3")))]
pub mod dac;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
        use core::marker::PhantomData;

        use crate::pac::SENS;

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

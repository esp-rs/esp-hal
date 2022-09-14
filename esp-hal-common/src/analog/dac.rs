use crate::pac::{RTCIO, SENS};

pub trait DAC {
    fn write(&mut self, value: u8);
}

#[doc(hidden)]
pub trait DAC1Impl {
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

        let rtcio = unsafe { &*RTCIO::ptr() };

        rtcio.pad_dac1.modify(|_, w| {
            w.pdac1_dac_xpd_force().set_bit();
            w.pdac1_xpd_dac().set_bit()
        });

        self
    }

    fn write(&mut self, value: u8) {
        let rtcio = unsafe { &*RTCIO::ptr() };

        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_dac_ctrl2
            .modify(|_, w| w.dac_cw_en1().clear_bit());

        rtcio
            .pad_dac1
            .modify(|_, w| unsafe { w.pdac1_dac().bits(value) });
    }
}

#[doc(hidden)]
pub trait DAC2Impl {
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

        let rtcio = unsafe { &*RTCIO::ptr() };

        rtcio.pad_dac2.modify(|_, w| {
            w.pdac2_dac_xpd_force().set_bit();
            w.pdac2_xpd_dac().set_bit()
        });

        self
    }

    fn write(&mut self, value: u8) {
        let rtcio = unsafe { &*RTCIO::ptr() };

        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_dac_ctrl2
            .modify(|_, w| w.dac_cw_en2().clear_bit());

        rtcio
            .pad_dac2
            .modify(|_, w| unsafe { w.pdac2_dac().bits(value) });
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_dac {
    ($($number:literal => $gpio:ident,)+) => {
        use core::marker::PhantomData;
        use crate::gpio;

        $(
            paste! {
                pub use esp_hal_common::analog::dac::[<DAC $number Impl>];

                /// DAC channel
                pub struct [<DAC $number>] {
                    _private: PhantomData<()>,
                }

                impl [<DAC $number Impl>] for [<DAC $number>] {}

                impl [<DAC $number>] {
                    /// Constructs a new DAC instance
                    pub fn dac(
                        _dac: esp_hal_common::analog::[<DAC $number>],
                        _pin: gpio::$gpio<esp_hal_common::Analog>,
                    ) -> Result<Self, ()> {
                        let dac = Self {
                            _private: PhantomData,
                        }
                        .set_power();
                        Ok(dac)
                    }

                    /// Write the given value
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

pub use impl_dac;

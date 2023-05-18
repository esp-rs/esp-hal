use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};
#[cfg(esp32c3)]
use paste::paste;

#[cfg(esp32c3)]
use crate::analog::ADC2;
#[cfg(esp32c6)]
use crate::clock::clocks_ll::regi2c_write_mask;
#[cfg(esp32c3)]
use crate::efuse::Efuse;
#[cfg(not(esp32c6))]
use crate::rom::rom_i2c_writeReg_Mask;
use crate::{
    analog::ADC1,
    peripheral::PeripheralRef,
    peripherals::APB_SARADC,
    system::{Peripheral, PeripheralClockControl},
};

// constants taken from https://github.com/espressif/esp-idf/blob/045163a2ec99eb3cb7cc69e2763afd145156c4cf/components/soc/esp32s3/include/soc/regi2c_saradc.h
cfg_if::cfg_if! {
    if #[cfg(esp32c3)] {
        const I2C_SAR_ADC: u32 = 0x69;
        const I2C_SAR_ADC_HOSTID: u32 = 0;

        const ADC_SAR1_ENCAL_GND_ADDR: u32 = 0x7;
        const ADC_SAR1_ENCAL_GND_ADDR_MSB: u32 = 5;
        const ADC_SAR1_ENCAL_GND_ADDR_LSB: u32 = 5;

        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR: u32 = 0x1;
        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB: u32 = 0x3;
        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB: u32 = 0x0;

        const ADC_SAR1_INITIAL_CODE_LOW_ADDR: u32 = 0x0;
        const ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB: u32 = 0x7;
        const ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB: u32 = 0x0;

        const ADC_SAR1_DREF_ADDR: u32 = 0x2;
        const ADC_SAR1_DREF_ADDR_MSB: u32 = 0x6;
        const ADC_SAR1_DREF_ADDR_LSB: u32 = 0x4;

        const ADC_VAL_MASK: u32 = 0xfff;
        const ADC_CAL_VER_OFF: u32 = 128;
        const ADC_CAL_VER_LEN: u32 = 3;
        const ADC_CAL_DATA_COMP: u32 = 1000;
        const ADC_CAL_DATA_LEN: u32 = 10;

        const APB_SARADC1_ONETIME_SAMPLE: u32 = 1 << 31;
        const APB_SARADC2_ONETIME_SAMPLE: u32 = 1 << 30;
        const APB_SARADC_ONETIME_CHANNEL_M: u32 = 0xf << 25;
        const APB_SARADC_ONETIME_ATTEN_M: u32 = 1 << 23;
        const APB_SARADC_ONETIME_CHANNEL_S: u32 = 25;
        const APB_SARADC_ONETIME_ATTEN_S: u32 = 23;
        const APB_SARADC_ADC1_DONE_INT_ST: u32 = 1 << 31;
        const DR_REG_EFUSE_BASE: u32 = 0x6000_8800;
        const ADC_CAL_BASE_REG: u32 = DR_REG_EFUSE_BASE + 0x5c;
        const ADC_CAL_DATA_OFF: u32 = 148;
        const ADC_CAL_CNT_MAX: u16 = 32;
        const ADC_CAL_VOL_OFF: u32 = 188;
        const ADC_CAL_VOL_LEN: u32 = 10;
        const ADC_CAL_CHANNEL: u32 = 0xf;

        static mut G_CAL_DIGIT: u16 = 0;
    }
}

/// The sampling/readout resolution of the ADC
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Resolution {
    Resolution12Bit,
}

/// The attenuation of the ADC pin
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Attenuation {
    Attenuation0dB   = 0b00,
    Attenuation2p5dB = 0b01,
    Attenuation6dB   = 0b10,
    Attenuation11dB  = 0b11,
}

pub struct AdcPin<PIN, ADCI> {
    pub pin: PIN,
    _phantom: PhantomData<ADCI>,
}

impl<PIN: Channel<ADCI, ID = u8>, ADCI> Channel<ADCI> for AdcPin<PIN, ADCI> {
    type ID = u8;

    fn channel() -> Self::ID {
        PIN::channel()
    }
}

pub struct AdcConfig<ADCI> {
    pub resolution: Resolution,
    pub attenuations: [Option<Attenuation>; 5],
    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcConfig<ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn new() -> AdcConfig<ADCI> {
        Self::default()
    }

    pub fn enable_pin<PIN: Channel<ADCI, ID = u8>>(
        &mut self,
        pin: PIN,
        attenuation: Attenuation,
    ) -> AdcPin<PIN, ADCI> {
        self.attenuations[PIN::channel() as usize] = Some(attenuation);

        // calibrate raw values
        #[cfg(esp32c3)]
        Self::calibrate(attenuation);

        AdcPin {
            pin,
            _phantom: PhantomData::default(),
        }
    }

    #[cfg(esp32c3)]
    #[inline(always)]
    /// Set calibration parameter to ADC hardware
    fn adc_set_calibration(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        unsafe {
            crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_INITIAL_CODE_HIGH_ADDR, msb as u32);
            crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_INITIAL_CODE_LOW_ADDR, lsb as u32);
        }
    }

    #[cfg(esp32c3)]
    fn adc_samplecfg(channel: u32, atten: Attenuation) {
        let apb_saradc = unsafe { &*APB_SARADC::PTR };

        let mut regval = apb_saradc.onetime_sample.read().bits();
        regval &= !(APB_SARADC1_ONETIME_SAMPLE
            | APB_SARADC2_ONETIME_SAMPLE
            | APB_SARADC_ONETIME_CHANNEL_M
            | APB_SARADC_ONETIME_ATTEN_M);

        regval |= (channel << APB_SARADC_ONETIME_CHANNEL_S)
            | ((atten as u32) << APB_SARADC_ONETIME_ATTEN_S)
            | APB_SARADC1_ONETIME_SAMPLE;

        apb_saradc
            .onetime_sample
            .write(|w| unsafe { w.bits(regval) });
    }

    #[cfg(esp32c3)]
    fn adc_read() -> u32 {
        let mut regval;

        let apb_saradc = unsafe { &*APB_SARADC::PTR };
        // Trigger ADC sampling

        apb_saradc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().set_bit());

        // Wait until ADC1 sampling is done
        loop {
            regval = apb_saradc.int_st.read().bits();

            if regval & APB_SARADC_ADC1_DONE_INT_ST != 0 {
                break;
            }
        }

        let adc = apb_saradc.sar1data_status.read().bits() & ADC_VAL_MASK;

        // Disable ADC sampling

        apb_saradc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().clear_bit());

        // Clear ADC1 sampling done interrupt bit

        apb_saradc
            .int_clr
            .write(|w| w.apb_saradc1_done_int_clr().set_bit());

        adc
    }

    #[cfg(esp32c3)]
    pub fn calibrate(attent: Attenuation) {
        let cali_val: u16;
        let mut adc: u16;
        let mut adc_max: u16 = 0;
        let mut adc_min: u16 = u16::MAX;
        let mut adc_sum: u32 = 0;

        let mut regval = Efuse::read_efuse(ADC_CAL_BASE_REG, ADC_CAL_VER_OFF, ADC_CAL_VER_LEN);

        if regval == 1 {
            regval = Efuse::read_efuse(
                ADC_CAL_BASE_REG,
                ADC_CAL_DATA_OFF + (attent as u32 * 10),
                ADC_CAL_DATA_LEN,
            );
            cali_val = (regval + ADC_CAL_DATA_COMP) as u16;
        } else {
            // Enable Vdef
            unsafe {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_DREF_ADDR, 1);
            }

            // Start sampling
            Self::adc_samplecfg(ADC_CAL_CHANNEL, attent);

            // Enable internal connect GND (for calibration)
            unsafe {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_ENCAL_GND_ADDR, 1);
            }

            let max = |x, y| {
                if x >= y {
                    x
                } else {
                    y
                }
            };

            let min = |x, y| {
                if x <= y {
                    x
                } else {
                    y
                }
            };

            for _ in 0..ADC_CAL_CNT_MAX {
                Self::adc_set_calibration(0);
                adc = Self::adc_read() as u16;
                adc_sum += adc as u32;
                adc_max = max(adc, adc_max);
                adc_min = min(adc, adc_min);
            }

            cali_val = (adc_sum - adc_max as u32 - adc_min as u32) as u16 / (ADC_CAL_CNT_MAX - 2);

            unsafe {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_ENCAL_GND_ADDR, 0);
            }
        }

        // Set final calibration parameters
        Self::adc_set_calibration(cali_val);

        // Set calibration digital parameters
        regval = Efuse::read_efuse(
            ADC_CAL_BASE_REG,
            ADC_CAL_VOL_OFF + (attent as u32 * 10),
            ADC_CAL_VOL_LEN,
        );

        if regval & (1 << (ADC_CAL_VOL_LEN - 1)) == 0 {
            unsafe { G_CAL_DIGIT = 2000 - (regval & !(1 << (ADC_CAL_VOL_LEN - 1))) as u16 };
        } else {
            unsafe { G_CAL_DIGIT = 2000 + regval as u16 };
        }
    }
}

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        AdcConfig {
            resolution: Resolution::Resolution12Bit,
            attenuations: [None; 5],
            _phantom: PhantomData::default(),
        }
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    fn start_onetime_sample(channel: u8, attenuation: u8);

    fn is_done() -> bool;

    fn read_data() -> u16;

    fn reset();
}

impl RegisterAccess for ADC1 {
    fn start_onetime_sample(channel: u8, attenuation: u8) {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.onetime_sample.modify(|_, w| unsafe {
            w.saradc1_onetime_sample()
                .set_bit()
                .saradc_onetime_channel()
                .bits(channel)
                .saradc_onetime_atten()
                .bits(attenuation)
                .saradc_onetime_start()
                .set_bit()
        });
    }

    fn is_done() -> bool {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.int_raw.read().apb_saradc1_done_int_raw().bit()
    }

    fn read_data() -> u16 {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        (sar_adc.sar1data_status.read().apb_saradc1_data().bits() as u16) & 0xfff
    }

    fn reset() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .int_clr
            .write(|w| w.apb_saradc1_done_int_clr().set_bit());

        sar_adc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().clear_bit());
    }
}

#[cfg(esp32c3)]
impl RegisterAccess for ADC2 {
    fn start_onetime_sample(channel: u8, attenuation: u8) {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.onetime_sample.modify(|_, w| unsafe {
            w.saradc2_onetime_sample()
                .set_bit()
                .saradc_onetime_channel()
                .bits(channel)
                .saradc_onetime_atten()
                .bits(attenuation)
                .saradc_onetime_start()
                .set_bit()
        });
    }

    fn is_done() -> bool {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.int_raw.read().apb_saradc2_done_int_raw().bit()
    }

    fn read_data() -> u16 {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        (sar_adc.sar2data_status.read().apb_saradc2_data().bits() as u16) & 0xfff
    }

    fn reset() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .int_clr
            .write(|w| w.apb_saradc2_done_int_clr().set_bit());

        sar_adc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().clear_bit());
    }
}

pub struct ADC<'d, ADCI> {
    _adc: PeripheralRef<'d, ADCI>,
    attenuations: [Option<Attenuation>; 5],
    active_channel: Option<u8>,
}

impl<'d, ADCI> ADC<'d, ADCI>
where
    ADCI: RegisterAccess + 'd,
{
    pub fn adc(
        peripheral_clock_controller: &mut PeripheralClockControl,
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Result<Self, ()> {
        peripheral_clock_controller.enable(Peripheral::ApbSarAdc);

        let sar_adc = unsafe { &*APB_SARADC::PTR };
        sar_adc.ctrl.modify(|_, w| unsafe {
            w.saradc_start_force()
                .set_bit()
                .saradc_start()
                .set_bit()
                .saradc_sar_clk_gated()
                .set_bit()
                .saradc_xpd_sar_force()
                .bits(0b11)
        });
        let adc = ADC {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
        };

        Ok(adc)
    }
}

impl<'d, ADCI, WORD, PIN> OneShot<ADCI, WORD, AdcPin<PIN, ADCI>> for ADC<'d, ADCI>
where
    WORD: From<u16>,
    PIN: Channel<ADCI, ID = u8>,
    ADCI: RegisterAccess,
{
    type Error = ();

    fn read(&mut self, _pin: &mut AdcPin<PIN, ADCI>) -> nb::Result<WORD, Self::Error> {
        if self.attenuations[AdcPin::<PIN, ADCI>::channel() as usize] == None {
            panic!(
                "Channel {} is not configured reading!",
                AdcPin::<PIN, ADCI>::channel()
            );
        }

        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahead and check progress
            if active_channel != AdcPin::<PIN, ADCI>::channel() {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(AdcPin::<PIN, ADCI>::channel());

            let channel = self.active_channel.unwrap();
            let attenuation = self.attenuations[channel as usize].unwrap() as u8;
            ADCI::start_onetime_sample(channel, attenuation);
        }

        // Wait for ADC to finish conversion
        let conversion_finished = ADCI::is_done();
        if !conversion_finished {
            return Err(nb::Error::WouldBlock);
        }

        // Get converted value
        let converted_value = ADCI::read_data();
        ADCI::reset();

        // There is a hardware limitation. If the APB clock frequency is high, the step
        // of this reg signal: ``onetime_start`` may not be captured by the
        // ADC digital controller (when its clock frequency is too slow). A rough
        // estimate for this step should be at least 3 ADC digital controller
        // clock cycle.
        //
        // This limitation will be removed in hardware future versions.
        // We reset ``onetime_start`` in `reset` and assume enough time has passed until
        // the next sample is requested.

        // Mark that no conversions are currently in progress
        self.active_channel = None;

        Ok(converted_value.into())
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_adc_interface {
    ($adc:ident [
        $( ($pin:ident, $channel:expr) ,)+
    ]) => {

        $(
            impl Channel<$adc> for $pin<Analog> {
                type ID = u8;

                fn channel() -> u8 { $channel }
            }
        )+
    }
}

pub use impl_adc_interface;

#[cfg(esp32c2)]
pub mod implementation {
    //! Analog to digital (ADC) conversion support.
    //!
    //! This module provides functions for reading analog values from the
    //! analog to digital converter available on the ESP32-C2: `ADC1`.

    use embedded_hal::adc::Channel;

    use super::impl_adc_interface;
    pub use crate::analog::{adc::*, ADC1};
    use crate::gpio::*;

    impl_adc_interface! {
        ADC1 [
            (Gpio0, 0),
            (Gpio1, 1),
            (Gpio2, 2),
            (Gpio3, 3),
            (Gpio4, 4),
        ]
    }
}

#[cfg(esp32c3)]
pub mod implementation {
    //! Analog to digital (ADC) conversion support.
    //!
    //! This module provides functions for reading analog values from two
    //! analog to digital converters available on the ESP32-C3: `ADC1` and
    //! `ADC2`.

    use embedded_hal::adc::Channel;

    use super::impl_adc_interface;
    pub use crate::analog::{adc::*, ADC1, ADC2};
    use crate::gpio::*;

    impl_adc_interface! {
        ADC1 [
            (Gpio0, 0),
            (Gpio1, 1),
            (Gpio2, 2),
            (Gpio3, 3),
            (Gpio4, 4),
        ]
    }

    impl_adc_interface! {
        ADC2 [
            (Gpio5, 4),
        ]
    }
}

#[cfg(esp32c6)]
pub mod implementation {
    //! Analog to digital (ADC) conversion support.
    //!
    //! This module provides functions for reading analog values from one
    //! analog to digital converter available on the ESP32-C6: `ADC1`.

    use embedded_hal::adc::Channel;

    use super::impl_adc_interface;
    pub use crate::analog::{adc::*, ADC1};
    use crate::gpio::*;

    impl_adc_interface! {
        ADC1 [
            (Gpio0, 0),
            (Gpio1, 1),
            (Gpio2, 2),
            (Gpio3, 3),
            (Gpio4, 4),
            (Gpio5, 5),
            (Gpio6, 6),
        ]
    }
}

#[cfg(esp32h2)]
pub mod implementation {
    //! Analog to digital (ADC) conversion support.
    //!
    //! This module provides functions for reading analog values from one
    //! analog to digital converter available on the ESP32-H2: `ADC1`.

    use embedded_hal::adc::Channel;

    use super::impl_adc_interface;
    pub use crate::analog::{adc::*, ADC1};
    use crate::gpio::*;

    impl_adc_interface! {
        ADC1 [
            (Gpio1, 0),
            (Gpio2, 1),
            (Gpio3, 2),
            (Gpio4, 3),
            (Gpio5, 4),
        ]
    }
}

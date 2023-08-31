use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};

#[cfg(esp32c3)]
use crate::analog::ADC2;
#[cfg(any(esp32c6, esp32h2))]
use crate::clock::clocks_ll::regi2c_write_mask;
#[cfg(any(esp32c2, esp32c3, esp32c6))]
use crate::efuse::Efuse;
use crate::{
    analog::ADC1,
    peripheral::PeripheralRef,
    peripherals::APB_SARADC,
    system::{Peripheral, PeripheralClockControl},
};

#[cfg(any(esp32c2, esp32c3, esp32c6))]
mod cal_basic;
#[cfg(any(esp32c3, esp32c6))]
mod cal_curve;
#[cfg(any(esp32c2, esp32c3, esp32c6))]
mod cal_line;

#[cfg(any(esp32c2, esp32c3, esp32c6))]
pub use cal_basic::AdcCalBasic;
#[cfg(any(esp32c3, esp32c6))]
pub use cal_curve::{AdcCalCurve, AdcHasCurveCal};
#[cfg(any(esp32c2, esp32c3, esp32c6))]
pub use cal_line::{AdcCalLine, AdcHasLineCal};

pub use crate::analog::{AdcCalEfuse, AdcCalScheme};

// polyfill for c2 and c3
#[cfg(any(esp32c2, esp32c3))]
#[inline(always)]
fn regi2c_write_mask(block: u8, host_id: u8, reg_add: u8, msb: u8, lsb: u8, data: u8) {
    unsafe {
        crate::rom::rom_i2c_writeReg_Mask(
            block as _,
            host_id as _,
            reg_add as _,
            msb as _,
            lsb as _,
            data as _,
        );
    }
}

// Constants taken from:
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c2/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c3/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32c6/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32h2/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32h4/include/soc/regi2c_saradc.h
cfg_if::cfg_if! {
    if #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))] {
        const I2C_SAR_ADC: u8 = 0x69;
        const I2C_SAR_ADC_HOSTID: u8 = 0;

        const ADC_VAL_MASK: u16 = 0xfff;
        const ADC_CAL_CNT_MAX: u16 = 32;
        const ADC_CAL_CHANNEL: u16 = 15;

        const ADC_SAR1_ENCAL_GND_ADDR: u8 = 0x7;
        const ADC_SAR1_ENCAL_GND_ADDR_MSB: u8 = 5;
        const ADC_SAR1_ENCAL_GND_ADDR_LSB: u8 = 5;

        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR: u8 = 0x1;
        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB: u8 = 0x3;
        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB: u8 = 0x0;

        const ADC_SAR1_INITIAL_CODE_LOW_ADDR: u8 = 0x0;
        const ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB: u8 = 0x7;
        const ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB: u8 = 0x0;

        const ADC_SAR1_DREF_ADDR: u8 = 0x2;
        const ADC_SAR1_DREF_ADDR_MSB: u8 = 0x6;
        const ADC_SAR1_DREF_ADDR_LSB: u8 = 0x4;

        const ADC_SARADC1_ENCAL_REF_ADDR: u8 = 0x7;
        const ADC_SARADC1_ENCAL_REF_ADDR_MSB: u8 = 4;
        const ADC_SARADC1_ENCAL_REF_ADDR_LSB: u8 = 4;
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32c3)] {
        const ADC_SAR2_ENCAL_GND_ADDR: u8 = 0x7;
        const ADC_SAR2_ENCAL_GND_ADDR_MSB: u8 = 7;
        const ADC_SAR2_ENCAL_GND_ADDR_LSB: u8 = 7;

        const ADC_SAR2_INITIAL_CODE_HIGH_ADDR: u8 = 0x4;
        const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB: u8 = 0x3;
        const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB: u8 = 0x0;

        const ADC_SAR2_INITIAL_CODE_LOW_ADDR: u8 = 0x3;
        const ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB: u8 = 0x7;
        const ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB: u8 = 0x0;

        const ADC_SAR2_DREF_ADDR: u8 = 0x5;
        const ADC_SAR2_DREF_ADDR_MSB: u8 = 0x6;
        const ADC_SAR2_DREF_ADDR_LSB: u8 = 0x4;

        const ADC_SARADC2_ENCAL_REF_ADDR: u8 = 0x7;
        const ADC_SARADC2_ENCAL_REF_ADDR_MSB: u8 = 6;
        const ADC_SARADC2_ENCAL_REF_ADDR_LSB: u8 = 6;
    }
}

// The number of analog IO pins, and in turn the number of attentuations,
// depends on which chip is being used
cfg_if::cfg_if! {
    if #[cfg(esp32c6)] {
        const NUM_ATTENS: usize = 7;
    } else {
        const NUM_ATTENS: usize = 5;
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
    /// 0 dB attenuation, measurement range: 0 - 800 mV
    Attenuation0dB   = 0b00,
    /// 2.5 dB attenuation, measurement range: 0 - 1100 mV
    #[cfg(not(esp32c2))]
    Attenuation2p5dB = 0b01,
    /// 6 dB attenuation, measurement range: 0 - 1350 mV
    #[cfg(not(esp32c2))]
    Attenuation6dB   = 0b10,
    /// 11 dB attenuation, measurement range: 0 - 2600 mV
    Attenuation11dB  = 0b11,
}

impl Attenuation {
    /// List of all supported attenuations
    pub const ALL: &'static [Attenuation] = &[
        Attenuation::Attenuation0dB,
        #[cfg(not(esp32c2))]
        Attenuation::Attenuation2p5dB,
        #[cfg(not(esp32c2))]
        Attenuation::Attenuation6dB,
        Attenuation::Attenuation11dB,
    ];

    /// Reference voltage in millivolts
    ///
    /// Vref = 10 ^ (Att / 20) * Vref0
    /// where Vref0 = 1.1 V, Att - attenuation in dB
    ///
    /// To convert raw value to millivolts use formula:
    /// V = D * Vref / 2 ^ R
    /// where D - raw ADC value, R - resolution in bits
    pub const fn ref_mv(&self) -> u16 {
        match self {
            Attenuation::Attenuation0dB => 1100,
            #[cfg(not(esp32c2))]
            Attenuation::Attenuation2p5dB => 1467,
            #[cfg(not(esp32c2))]
            Attenuation::Attenuation6dB => 2195,
            Attenuation::Attenuation11dB => 3903,
        }
    }
}

pub struct AdcPin<PIN, ADCI, CS = ()> {
    pub pin: PIN,
    pub cal_scheme: CS,
    _phantom: PhantomData<ADCI>,
}

impl<PIN, ADCI, CS> Channel<ADCI> for AdcPin<PIN, ADCI, CS>
where
    PIN: Channel<ADCI, ID = u8>,
{
    type ID = u8;

    fn channel() -> Self::ID {
        PIN::channel()
    }
}

pub struct AdcConfig<ADCI> {
    pub resolution: Resolution,
    pub attenuations: [Option<Attenuation>; NUM_ATTENS],
    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcConfig<ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn new() -> AdcConfig<ADCI> {
        Self::default()
    }

    pub fn enable_pin<PIN>(&mut self, pin: PIN, attenuation: Attenuation) -> AdcPin<PIN, ADCI>
    where
        PIN: Channel<ADCI, ID = u8>,
    {
        self.attenuations[PIN::channel() as usize] = Some(attenuation);

        AdcPin {
            pin,
            cal_scheme: AdcCalScheme::<()>::new_cal(attenuation),
            _phantom: PhantomData::default(),
        }
    }

    pub fn enable_pin_with_cal<PIN, CS>(
        &mut self,
        pin: PIN,
        attenuation: Attenuation,
    ) -> AdcPin<PIN, ADCI, CS>
    where
        ADCI: CalibrationAccess,
        PIN: Channel<ADCI, ID = u8>,
        CS: AdcCalScheme<ADCI>,
    {
        self.attenuations[PIN::channel() as usize] = Some(attenuation);

        AdcPin {
            pin,
            cal_scheme: CS::new_cal(attenuation),
            _phantom: PhantomData::default(),
        }
    }

    /// Calibrate ADC with specified attenuation and voltage source
    pub fn adc_calibrate(atten: Attenuation, source: AdcCalSource) -> u16
    where
        ADCI: CalibrationAccess,
    {
        let mut adc_max: u16 = 0;
        let mut adc_min: u16 = u16::MAX;
        let mut adc_sum: u32 = 0;

        ADCI::enable_vdef(true);

        // Start sampling
        ADCI::config_onetime_sample(ADC_CAL_CHANNEL as u8, atten as u8);

        // Connect calibration source
        ADCI::connect_cal(source, true);

        for _ in 0..ADC_CAL_CNT_MAX {
            ADCI::set_init_code(0);

            // Trigger ADC sampling
            ADCI::start_onetime_sample();

            // Wait until ADC1 sampling is done
            while !ADCI::is_done() {}

            let adc = ADCI::read_data() & ADC_VAL_MASK;

            ADCI::reset();

            adc_sum += adc as u32;
            adc_max = adc.max(adc_max);
            adc_min = adc.min(adc_min);
        }

        let cal_val = (adc_sum - adc_max as u32 - adc_min as u32) as u16 / (ADC_CAL_CNT_MAX - 2);

        // Disconnect calibration source
        ADCI::connect_cal(source, false);

        cal_val
    }
}

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        AdcConfig {
            resolution: Resolution::Resolution12Bit,
            attenuations: [None; NUM_ATTENS],
            _phantom: PhantomData::default(),
        }
    }
}

#[derive(Clone, Copy)]
pub enum AdcCalSource {
    Gnd,
    Ref,
}

#[doc(hidden)]
pub trait RegisterAccess {
    /// Configure onetime sampling parameters
    fn config_onetime_sample(channel: u8, attenuation: u8);

    /// Start onetime sampling
    fn start_onetime_sample();

    /// Check if sampling is done
    fn is_done() -> bool;

    /// Read sample data
    fn read_data() -> u16;

    /// Reset flags
    fn reset();

    /// Set calibration parameter to ADC hardware
    fn set_init_code(data: u16);
}

pub trait CalibrationAccess: RegisterAccess {
    const ADC_CAL_CNT_MAX: u16;
    const ADC_CAL_CHANNEL: u16;
    const ADC_VAL_MASK: u16;

    fn enable_vdef(enable: bool);

    /// Enable internal calibration voltage source
    fn connect_cal(source: AdcCalSource, enable: bool);
}

impl RegisterAccess for ADC1 {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.onetime_sample.modify(|_, w| unsafe {
            w.saradc1_onetime_sample()
                .set_bit()
                .saradc_onetime_channel()
                .bits(channel)
                .saradc_onetime_atten()
                .bits(attenuation)
        });
    }

    fn start_onetime_sample() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().set_bit());
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

        // Clear ADC1 sampling done interrupt bit
        sar_adc
            .int_clr
            .write(|w| w.apb_saradc1_done_int_clr().set_bit());

        // Disable ADC sampling
        sar_adc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().clear_bit());
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB,
            ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB,
            msb as _,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB,
            ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB,
            lsb as _,
        );
    }
}

impl CalibrationAccess for ADC1 {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        let value = enable as _;
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR1_DREF_ADDR,
            ADC_SAR1_DREF_ADDR_MSB,
            ADC_SAR1_DREF_ADDR_LSB,
            value,
        );
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        let value = enable as _;
        match source {
            AdcCalSource::Gnd => regi2c_write_mask(
                I2C_SAR_ADC,
                I2C_SAR_ADC_HOSTID,
                ADC_SAR1_ENCAL_GND_ADDR,
                ADC_SAR1_ENCAL_GND_ADDR_MSB,
                ADC_SAR1_ENCAL_GND_ADDR_LSB,
                value,
            ),
            AdcCalSource::Ref => regi2c_write_mask(
                I2C_SAR_ADC,
                I2C_SAR_ADC_HOSTID,
                ADC_SARADC1_ENCAL_REF_ADDR,
                ADC_SARADC1_ENCAL_REF_ADDR_MSB,
                ADC_SARADC1_ENCAL_REF_ADDR_LSB,
                value,
            ),
        }
    }
}

#[cfg(esp32c3)]
impl RegisterAccess for ADC2 {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.onetime_sample.modify(|_, w| unsafe {
            w.saradc2_onetime_sample()
                .set_bit()
                .saradc_onetime_channel()
                .bits(channel)
                .saradc_onetime_atten()
                .bits(attenuation)
        });
    }

    fn start_onetime_sample() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .onetime_sample
            .modify(|_, w| w.saradc_onetime_start().set_bit());
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

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB,
            ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB,
            msb as _,
        );
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB,
            ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB,
            lsb as _,
        );
    }
}

#[cfg(esp32c3)]
impl CalibrationAccess for ADC2 {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        let value = enable as _;
        regi2c_write_mask(
            I2C_SAR_ADC,
            I2C_SAR_ADC_HOSTID,
            ADC_SAR2_DREF_ADDR,
            ADC_SAR2_DREF_ADDR_MSB,
            ADC_SAR2_DREF_ADDR_LSB,
            value,
        );
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        let value = enable as _;
        match source {
            AdcCalSource::Gnd => regi2c_write_mask(
                I2C_SAR_ADC,
                I2C_SAR_ADC_HOSTID,
                ADC_SAR2_ENCAL_GND_ADDR,
                ADC_SAR2_ENCAL_GND_ADDR_MSB,
                ADC_SAR2_ENCAL_GND_ADDR_LSB,
                value,
            ),
            AdcCalSource::Ref => regi2c_write_mask(
                I2C_SAR_ADC,
                I2C_SAR_ADC_HOSTID,
                ADC_SARADC2_ENCAL_REF_ADDR,
                ADC_SARADC2_ENCAL_REF_ADDR_MSB,
                ADC_SARADC2_ENCAL_REF_ADDR_LSB,
                value,
            ),
        }
    }
}

pub struct ADC<'d, ADCI> {
    _adc: PeripheralRef<'d, ADCI>,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
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

#[cfg(any(esp32c2, esp32c3, esp32c6))]
impl AdcCalEfuse for ADC1 {
    fn get_init_code(atten: Attenuation) -> Option<u16> {
        Efuse::get_rtc_calib_init_code(1, atten)
    }

    fn get_cal_mv(atten: Attenuation) -> u16 {
        Efuse::get_rtc_calib_cal_mv(1, atten)
    }

    fn get_cal_code(atten: Attenuation) -> Option<u16> {
        Efuse::get_rtc_calib_cal_code(1, atten)
    }
}

#[cfg(esp32c3)]
impl AdcCalEfuse for ADC2 {
    fn get_init_code(atten: Attenuation) -> Option<u16> {
        Efuse::get_rtc_calib_init_code(2, atten)
    }

    fn get_cal_mv(atten: Attenuation) -> u16 {
        Efuse::get_rtc_calib_cal_mv(2, atten)
    }

    fn get_cal_code(atten: Attenuation) -> Option<u16> {
        Efuse::get_rtc_calib_cal_code(2, atten)
    }
}

impl<'d, ADCI, WORD, PIN, CS> OneShot<ADCI, WORD, AdcPin<PIN, ADCI, CS>> for ADC<'d, ADCI>
where
    WORD: From<u16>,
    PIN: Channel<ADCI, ID = u8>,
    ADCI: RegisterAccess,
    CS: AdcCalScheme<ADCI>,
{
    type Error = ();

    fn read(&mut self, pin: &mut AdcPin<PIN, ADCI, CS>) -> nb::Result<WORD, Self::Error> {
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

            // Set ADC unit calibration according used scheme for pin
            ADCI::set_init_code(pin.cal_scheme.adc_cal());

            let channel = self.active_channel.unwrap();
            let attenuation = self.attenuations[channel as usize].unwrap() as u8;
            ADCI::config_onetime_sample(channel, attenuation);
            ADCI::start_onetime_sample();
        }

        // Wait for ADC to finish conversion
        let conversion_finished = ADCI::is_done();
        if !conversion_finished {
            return Err(nb::Error::WouldBlock);
        }

        // Get converted value
        let converted_value = ADCI::read_data();
        ADCI::reset();

        // Postprocess converted value according to calibration scheme used for pin
        let converted_value = pin.cal_scheme.adc_val(converted_value);

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
    //! # Analog to digital (ADC) conversion support.
    //!
    //! ## Overview
    //! The `ADC` module in the `analog` driver enables users to perform
    //! analog-to-digital conversions, allowing them to measure real-world
    //! analog signals with high accuracy.
    //!
    //! This module provides functions for reading analog values from the
    //! analog to digital converter available on the ESP32-C2: `ADC1`.
    //!
    //! ## Example
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

    use embedded_hal::adc::Channel;

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
    //! # Analog to digital (ADC) conversion support.
    //!
    //! ## Overview
    //! The `ADC` module in the `analog` driver enables users to perform
    //! analog-to-digital conversions, allowing them to measure real-world
    //! analog signals with high accuracy.
    //!
    //! This module provides functions for reading analog values from the
    //! analog to digital converter available on the ESP32-C3: `ADC1` and
    //! `ADC2`.
    //!
    //! ## Example
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

    use embedded_hal::adc::Channel;

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
            (Gpio5, 0),
        ]
    }
}

#[cfg(esp32c6)]
pub mod implementation {
    //! # Analog to digital (ADC) conversion support.
    //!
    //! ## Overview
    //! The `ADC` module in the `analog` driver enables users to perform
    //! analog-to-digital conversions, allowing them to measure real-world
    //! analog signals with high accuracy.
    //!
    //! This module provides functions for reading analog values from the
    //! analog to digital converter available on the ESP32-C6: `ADC1`.
    //!
    //! ## Example
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

    use embedded_hal::adc::Channel;

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
    //! # Analog to digital (ADC) conversion support.
    //!
    //! ## Overview
    //! The `ADC` module in the `analog` driver enables users to perform
    //! analog-to-digital conversions, allowing them to measure real-world
    //! analog signals with high accuracy.
    //!
    //! This module provides functions for reading analog values from the
    //! analog to digital converter available on the  ESP32-H2: `ADC1`.
    //!
    //! ## Example
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

    use embedded_hal::adc::Channel;

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

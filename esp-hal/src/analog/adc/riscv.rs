#[cfg(not(esp32h2))]
pub use self::calibration::*;
use super::{AdcCalSource, AdcConfig, Attenuation};
#[cfg(any(esp32c6, esp32h2))]
use crate::clock::clocks_ll::regi2c_write_mask;
#[cfg(any(esp32c2, esp32c3, esp32c6))]
use crate::efuse::Efuse;
use crate::{
    peripheral::PeripheralRef,
    peripherals::APB_SARADC,
    system::{Peripheral, PeripheralClockControl},
};

mod calibration;

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
        pub(super) const NUM_ATTENS: usize = 7;
    } else {
        pub(super) const NUM_ATTENS: usize = 5;
    }
}

/// The sampling/readout resolution of the ADC.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Resolution {
    /// 12-bit resolution
    #[default]
    Resolution12Bit,
}

impl<ADCI> AdcConfig<ADCI>
where
    ADCI: RegisterAccess,
{
    /// Calibrate ADC with specified attenuation and voltage source
    pub fn adc_calibrate(atten: Attenuation, source: AdcCalSource) -> u16
    where
        ADCI: super::CalibrationAccess,
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

impl RegisterAccess for crate::peripherals::ADC1 {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.onetime_sample().modify(|_, w| unsafe {
            w.saradc1_onetime_sample().set_bit();
            w.onetime_channel().bits(channel);
            w.onetime_atten().bits(attenuation)
        });
    }

    fn start_onetime_sample() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .onetime_sample()
            .modify(|_, w| w.onetime_start().set_bit());
    }

    fn is_done() -> bool {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.int_raw().read().adc1_done().bit()
    }

    fn read_data() -> u16 {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        (sar_adc.sar1data_status().read().saradc1_data().bits() as u16) & 0xfff
    }

    fn reset() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        // Clear ADC1 sampling done interrupt bit
        sar_adc
            .int_clr()
            .write(|w| w.adc1_done().clear_bit_by_one());

        // Disable ADC sampling
        sar_adc
            .onetime_sample()
            .modify(|_, w| w.onetime_start().clear_bit());
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

impl super::CalibrationAccess for crate::peripherals::ADC1 {
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
impl RegisterAccess for crate::peripherals::ADC2 {
    fn config_onetime_sample(channel: u8, attenuation: u8) {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.onetime_sample().modify(|_, w| unsafe {
            w.saradc2_onetime_sample().set_bit();
            w.onetime_channel().bits(channel);
            w.onetime_atten().bits(attenuation)
        });
    }

    fn start_onetime_sample() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .onetime_sample()
            .modify(|_, w| w.onetime_start().set_bit());
    }

    fn is_done() -> bool {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc.int_raw().read().adc2_done().bit()
    }

    fn read_data() -> u16 {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        (sar_adc.sar2data_status().read().saradc2_data().bits() as u16) & 0xfff
    }

    fn reset() {
        let sar_adc = unsafe { &*APB_SARADC::PTR };

        sar_adc
            .int_clr()
            .write(|w| w.adc2_done().clear_bit_by_one());

        sar_adc
            .onetime_sample()
            .modify(|_, w| w.onetime_start().clear_bit());
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
impl super::CalibrationAccess for crate::peripherals::ADC2 {
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

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADCI> {
    _adc: PeripheralRef<'d, ADCI>,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
    active_channel: Option<u8>,
}

impl<'d, ADCI> Adc<'d, ADCI>
where
    ADCI: RegisterAccess + 'd,
{
    /// Configure a given ADC instance using the provided configuration, and
    /// initialize the ADC for use
    pub fn new(
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Self {
        if PeripheralClockControl::enable(Peripheral::ApbSarAdc, true) {
            PeripheralClockControl::reset(Peripheral::ApbSarAdc);
        }

        unsafe { &*APB_SARADC::PTR }.ctrl().modify(|_, w| unsafe {
            w.start_force().set_bit();
            w.start().set_bit();
            w.sar_clk_gated().set_bit();
            w.xpd_sar_force().bits(0b11)
        });

        Adc {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
        }
    }

    /// Request that the ADC begin a conversion on the specified pin
    ///
    /// This method takes an [AdcPin](super::AdcPin) reference, as it is
    /// expected that the ADC will be able to sample whatever channel
    /// underlies the pin.
    pub fn read_oneshot<PIN, CS>(
        &mut self,
        pin: &mut super::AdcPin<PIN, ADCI, CS>,
    ) -> nb::Result<u16, ()>
    where
        PIN: super::AdcChannel,
        CS: super::AdcCalScheme<ADCI>,
    {
        if self.attenuations[PIN::CHANNEL as usize].is_none() {
            panic!("Channel {} is not configured reading!", PIN::CHANNEL);
        }

        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahead and check progress
            if active_channel != PIN::CHANNEL {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(PIN::CHANNEL);

            // Set ADC unit calibration according used scheme for pin
            ADCI::set_init_code(pin.cal_scheme.adc_cal());

            let channel = self.active_channel.unwrap();
            let attenuation = self.attenuations[channel as usize].unwrap() as u8;
            ADCI::config_onetime_sample(channel, attenuation);
            ADCI::start_onetime_sample();

            // see https://github.com/espressif/esp-idf/blob/b4268c874a4cf8fcf7c0c4153cffb76ad2ddda4e/components/hal/adc_oneshot_hal.c#L105-L107
            // the delay might be a bit generous but longer delay seem to not cause problems
            #[cfg(esp32c6)]
            {
                crate::rom::ets_delay_us(40);
                ADCI::start_onetime_sample();
            }
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

        Ok(converted_value)
    }
}

impl<ADCI> Drop for Adc<'_, ADCI> {
    fn drop(&mut self) {
        PeripheralClockControl::enable(Peripheral::ApbSarAdc, false);
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6))]
impl super::AdcCalEfuse for crate::peripherals::ADC1 {
    fn init_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_init_code(1, atten)
    }

    fn cal_mv(atten: Attenuation) -> u16 {
        Efuse::rtc_calib_cal_mv(1, atten)
    }

    fn cal_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_cal_code(1, atten)
    }
}

#[cfg(esp32c3)]
impl super::AdcCalEfuse for crate::peripherals::ADC2 {
    fn init_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_init_code(2, atten)
    }

    fn cal_mv(atten: Attenuation) -> u16 {
        Efuse::rtc_calib_cal_mv(2, atten)
    }

    fn cal_code(atten: Attenuation) -> Option<u16> {
        Efuse::rtc_calib_cal_code(2, atten)
    }
}

impl<ADCI, PIN, CS> embedded_hal_02::adc::OneShot<ADCI, u16, super::AdcPin<PIN, ADCI, CS>>
    for Adc<'_, ADCI>
where
    PIN: embedded_hal_02::adc::Channel<ADCI, ID = u8> + super::AdcChannel,
    ADCI: RegisterAccess,
    CS: super::AdcCalScheme<ADCI>,
{
    type Error = ();

    fn read(&mut self, pin: &mut super::AdcPin<PIN, ADCI, CS>) -> nb::Result<u16, Self::Error> {
        self.read_oneshot(pin)
    }
}

#[cfg(esp32c2)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<0>, 0),
            (GpioPin<1>, 1),
            (GpioPin<2>, 2),
            (GpioPin<3>, 3),
            (GpioPin<4>, 4),
        ]
    }
}

#[cfg(esp32c3)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<0>, 0),
            (GpioPin<1>, 1),
            (GpioPin<2>, 2),
            (GpioPin<3>, 3),
            (GpioPin<4>, 4),
        ]
    }

    crate::analog::adc::impl_adc_interface! {
        ADC2 [
            (GpioPin<5>, 0),
        ]
    }
}

#[cfg(esp32c6)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<0>, 0),
            (GpioPin<1>, 1),
            (GpioPin<2>, 2),
            (GpioPin<3>, 3),
            (GpioPin<4>, 4),
            (GpioPin<5>, 5),
            (GpioPin<6>, 6),
        ]
    }
}

#[cfg(esp32h2)]
mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<1>, 0),
            (GpioPin<2>, 1),
            (GpioPin<3>, 2),
            (GpioPin<4>, 3),
            (GpioPin<5>, 4),
        ]
    }
}

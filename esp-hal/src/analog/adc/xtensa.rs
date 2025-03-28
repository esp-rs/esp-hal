use core::marker::PhantomData;

#[cfg(esp32s3)]
pub use self::calibration::*;
use super::{AdcCalScheme, AdcCalSource, AdcChannel, AdcConfig, AdcPin, Attenuation};
#[cfg(esp32s3)]
use crate::efuse::Efuse;
use crate::{
    peripherals::{APB_SARADC, SENS},
    soc::regi2c,
    system::{GenericPeripheralGuard, Peripheral},
};

mod calibration;

pub(super) const NUM_ATTENS: usize = 10;

cfg_if::cfg_if! {
    if #[cfg(esp32s3)] {
        const ADC_VAL_MASK: u16 = 0xfff;
        const ADC_CAL_CNT_MAX: u16 = 32;
        const ADC_CAL_CHANNEL: u16 = 15;
    }
}

/// The sampling/readout resolution of the ADC.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "peripheral is unstable")]
pub enum Resolution {
    /// 13-bit resolution
    #[default]
    Resolution13Bit,
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
        ADCI::set_en_pad(ADCI::ADC_CAL_CHANNEL as u8);
        ADCI::set_attenuation(ADCI::ADC_CAL_CHANNEL as usize, atten as u8);

        // Connect calibration source
        ADCI::connect_cal(source, true);

        ADCI::set_init_code(0);

        for _ in 0..ADCI::ADC_CAL_CNT_MAX {
            // Trigger ADC sampling
            ADCI::start_sample();

            // Wait until ADC1 sampling is done
            while !ADCI::is_done() {}

            let adc = ADCI::read_data() & ADCI::ADC_VAL_MASK;

            ADCI::reset();

            adc_sum += adc as u32;
            adc_max = adc.max(adc_max);
            adc_min = adc.min(adc_min);
        }

        let cal_val =
            (adc_sum - adc_max as u32 - adc_min as u32) as u16 / (ADCI::ADC_CAL_CNT_MAX - 2);

        // Disconnect calibration source
        ADCI::connect_cal(source, false);

        cal_val
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    fn set_attenuation(channel: usize, attenuation: u8);

    fn clear_dig_force();

    fn set_start_force();

    fn set_en_pad_force();

    fn set_en_pad(channel: u8);

    fn clear_start_sample();

    fn start_sample();

    /// Check if sampling is done
    fn is_done() -> bool;

    /// Read sample data
    fn read_data() -> u16;

    /// Set calibration parameter to ADC hardware
    fn set_init_code(data: u16);

    /// Reset flags
    fn reset();
}

impl RegisterAccess for crate::peripherals::ADC1<'_> {
    fn set_attenuation(channel: usize, attenuation: u8) {
        SENS::regs().sar_atten1().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation & 0b11) as u32) << (channel * 2));

            unsafe { w.sar1_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        SENS::regs()
            .sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().clear_bit());
    }

    fn set_start_force() {
        SENS::regs()
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_force().set_bit());
    }

    fn set_en_pad_force() {
        SENS::regs()
            .sar_meas1_ctrl2()
            .modify(|_, w| w.sar1_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        SENS::regs()
            .sar_meas1_ctrl2()
            .modify(|_, w| unsafe { w.sar1_en_pad().bits(1 << channel) });
    }

    fn clear_start_sample() {
        SENS::regs()
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }

    fn start_sample() {
        SENS::regs()
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().set_bit());
    }

    fn is_done() -> bool {
        SENS::regs()
            .sar_meas1_ctrl2()
            .read()
            .meas1_done_sar()
            .bit_is_set()
    }

    fn read_data() -> u16 {
        SENS::regs()
            .sar_meas1_ctrl2()
            .read()
            .meas1_data_sar()
            .bits()
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR1_INITIAL_CODE_HIGH.write_field(msb);
        regi2c::ADC_SAR1_INITIAL_CODE_LOW.write_field(lsb);
    }

    fn reset() {
        let adc = APB_SARADC::regs();
        let sensors = SENS::regs();

        adc.int_clr().write(|w| w.adc1_done().clear_bit_by_one());

        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }
}

#[cfg(esp32s3)]
impl super::CalibrationAccess for crate::peripherals::ADC1<'_> {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR1_DREF.write_field(enable as u8);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => regi2c::ADC_SAR1_ENCAL_GND.write_field(enable as u8),
            AdcCalSource::Ref => regi2c::ADC_SAR1_ENCAL_REF.write_field(enable as u8),
        }
    }
}

impl RegisterAccess for crate::peripherals::ADC2<'_> {
    fn set_attenuation(channel: usize, attenuation: u8) {
        SENS::regs().sar_atten2().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation & 0b11) as u32) << (channel * 2));

            unsafe { w.sar2_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        SENS::regs()
            .sar_meas2_mux()
            .modify(|_, w| w.sar2_rtc_force().set_bit());

        APB_SARADC::regs()
            .arb_ctrl()
            .modify(|_, w| w.rtc_force().set_bit());
    }

    fn set_start_force() {
        SENS::regs()
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_force().set_bit());
    }

    fn set_en_pad_force() {
        SENS::regs()
            .sar_meas2_ctrl2()
            .modify(|_, w| w.sar2_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        SENS::regs()
            .sar_meas2_ctrl2()
            .modify(|_, w| unsafe { w.sar2_en_pad().bits(1 << channel) });
    }

    fn clear_start_sample() {
        SENS::regs()
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }

    fn start_sample() {
        SENS::regs()
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().set_bit());
    }

    fn is_done() -> bool {
        SENS::regs()
            .sar_meas2_ctrl2()
            .read()
            .meas2_done_sar()
            .bit_is_set()
    }

    fn read_data() -> u16 {
        SENS::regs()
            .sar_meas2_ctrl2()
            .read()
            .meas2_data_sar()
            .bits()
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        regi2c::ADC_SAR2_INITIAL_CODE_HIGH.write_field(msb);
        regi2c::ADC_SAR2_INITIAL_CODE_LOW.write_field(lsb);
    }

    fn reset() {
        let adc = APB_SARADC::regs();
        let sensors = SENS::regs();

        adc.int_clr().write(|w| w.adc2_done().clear_bit_by_one());

        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }
}

#[cfg(esp32s3)]
impl super::CalibrationAccess for crate::peripherals::ADC2<'_> {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        regi2c::ADC_SAR2_DREF.write_field(enable as u8);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => regi2c::ADC_SAR2_ENCAL_GND.write_field(enable as u8),
            AdcCalSource::Ref => regi2c::ADC_SAR2_ENCAL_REF.write_field(enable as u8),
        }
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADC, Dm: crate::DriverMode> {
    _adc: ADC,
    active_channel: Option<u8>,
    last_init_code: u16,
    _guard: GenericPeripheralGuard<{ Peripheral::ApbSarAdc as u8 }>,
    _phantom: PhantomData<(Dm, &'d mut ())>,
}

impl<'d, ADCI> Adc<'d, ADCI, crate::Blocking>
where
    ADCI: RegisterAccess + 'd,
{
    /// Configure a given ADC instance using the provided configuration, and
    /// initialize the ADC for use
    pub fn new(adc_instance: ADCI, config: AdcConfig<ADCI>) -> Self {
        let guard = GenericPeripheralGuard::new();
        let sensors = SENS::regs();

        // Set attenuation for pins
        let attenuations = config.attenuations;

        for (channel, attenuation) in attenuations.iter().enumerate() {
            if let Some(attenuation) = attenuation {
                ADCI::set_attenuation(channel, *attenuation as u8);
            }
        }

        // Set controller to RTC
        ADCI::clear_dig_force();
        ADCI::set_start_force();
        ADCI::set_en_pad_force();
        sensors
            .sar_hall_ctrl()
            .modify(|_, w| w.xpd_hall_force().set_bit());
        sensors
            .sar_hall_ctrl()
            .modify(|_, w| w.hall_phase_force().set_bit());

        // Set power to SW power on
        #[cfg(esp32s2)]
        sensors
            .sar_meas1_ctrl1()
            .modify(|_, w| w.rtc_saradc_clkgate_en().set_bit());

        #[cfg(esp32s3)]
        sensors
            .sar_peri_clk_gate_conf()
            .modify(|_, w| w.saradc_clk_en().set_bit());

        sensors
            .sar_power_xpd_sar()
            .modify(|_, w| w.sarclk_en().set_bit());

        sensors
            .sar_power_xpd_sar()
            .modify(|_, w| unsafe { w.force_xpd_sar().bits(0b11) });

        // disable AMP
        sensors
            .sar_meas1_ctrl1()
            .modify(|_, w| unsafe { w.force_xpd_amp().bits(0b11) });
        sensors
            .sar_amp_ctrl3()
            .modify(|_, w| unsafe { w.amp_rst_fb_fsm().bits(0) });
        sensors
            .sar_amp_ctrl3()
            .modify(|_, w| unsafe { w.amp_short_ref_fsm().bits(0) });
        sensors
            .sar_amp_ctrl3()
            .modify(|_, w| unsafe { w.amp_short_ref_gnd_fsm().bits(0) });
        sensors
            .sar_amp_ctrl1()
            .modify(|_, w| unsafe { w.sar_amp_wait1().bits(1) });
        sensors
            .sar_amp_ctrl1()
            .modify(|_, w| unsafe { w.sar_amp_wait2().bits(1) });
        sensors
            .sar_amp_ctrl2()
            .modify(|_, w| unsafe { w.sar_amp_wait3().bits(1) });

        Adc {
            _adc: adc_instance,
            active_channel: None,
            last_init_code: 0,
            _guard: guard,
            _phantom: PhantomData,
        }
    }

    /// Start and wait for a conversion on the specified pin and return the
    /// result
    pub fn read_blocking<PIN, CS>(&mut self, pin: &mut AdcPin<PIN, ADCI, CS>) -> u16
    where
        PIN: AdcChannel,
        CS: AdcCalScheme<ADCI>,
    {
        self.start_sample(pin);

        // Wait for ADC to finish conversion
        while !ADCI::is_done() {}

        // Get converted value
        let converted_value = ADCI::read_data();
        ADCI::reset();

        // Postprocess converted value according to calibration scheme used for pin
        pin.cal_scheme.adc_val(converted_value)
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

            self.start_sample(pin);
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

        // Mark that no conversions are currently in progress
        self.active_channel = None;

        Ok(converted_value)
    }

    fn start_sample<PIN, CS>(&mut self, pin: &mut AdcPin<PIN, ADCI, CS>)
    where
        PIN: AdcChannel,
        CS: AdcCalScheme<ADCI>,
    {
        // Set ADC unit calibration according used scheme for pin
        let init_code = pin.cal_scheme.adc_cal();
        if self.last_init_code != init_code {
            ADCI::set_init_code(init_code);
            self.last_init_code = init_code;
        }

        ADCI::set_en_pad(PIN::CHANNEL);

        ADCI::clear_start_sample();
        ADCI::start_sample();
    }
}

#[cfg(esp32s3)]
impl super::AdcCalEfuse for crate::peripherals::ADC1<'_> {
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

#[cfg(esp32s3)]
impl super::AdcCalEfuse for crate::peripherals::ADC2<'_> {
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

mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (GpioPin<'_, 1>,  0),
            (GpioPin<'_, 2>,  1),
            (GpioPin<'_, 3>,  2),
            (GpioPin<'_, 4>,  3),
            (GpioPin<'_, 5>,  4),
            (GpioPin<'_, 6>,  5),
            (GpioPin<'_, 7>,  6),
            (GpioPin<'_, 8>,  7),
            (GpioPin<'_, 9>,  8),
            (GpioPin<'_, 10>, 9),
        ]
    }

    crate::analog::adc::impl_adc_interface! {
        ADC2 [
            (GpioPin<'_, 11>, 0),
            (GpioPin<'_, 12>, 1),
            (GpioPin<'_, 13>, 2),
            (GpioPin<'_, 14>, 3),
            (GpioPin<'_, 15>, 4),
            (GpioPin<'_, 16>, 5),
            (GpioPin<'_, 17>, 6),
            (GpioPin<'_, 18>, 7),
            (GpioPin<'_, 19>, 8),
            (GpioPin<'_, 20>, 9),
        ]
    }
}

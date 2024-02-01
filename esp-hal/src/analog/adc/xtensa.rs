use core::marker::PhantomData;

pub use self::calibration::*;
#[cfg(esp32s3)]
use super::AdcCalEfuse;
use super::{AdcCalScheme, AdcCalSource, AdcChannel, Attenuation};
#[cfg(esp32s3)]
use crate::efuse::Efuse;
use crate::{
    peripheral::PeripheralRef,
    peripherals::{APB_SARADC, SENS},
};

mod calibration;

// Constants taken from:
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32s2/include/soc/regi2c_saradc.h
// https://github.com/espressif/esp-idf/blob/903af13e8/components/soc/esp32s3/include/soc/regi2c_saradc.h

cfg_if::cfg_if! {
    if #[cfg(any(esp32s2, esp32s3))] {
        const I2C_SAR_ADC: u8 = 0x69;
        const I2C_SAR_ADC_HOSTID: u8 = 1;

        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR: u8 = 0x1;
        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB: u8 = 0x3;
        const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB: u8 = 0x0;

        const ADC_SAR1_INITIAL_CODE_LOW_ADDR: u8 = 0x0;
        const ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB: u8 = 0x7;
        const ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB: u8 = 0x0;

        const ADC_SAR2_INITIAL_CODE_HIGH_ADDR: u8 = 0x4;
        const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB: u8 = 0x3;
        const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB: u8 = 0x0;

        const ADC_SAR2_INITIAL_CODE_LOW_ADDR: u8 = 0x3;
        const ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB: u8 = 0x7;
        const ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB: u8 = 0x0;
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32s3)] {
        const ADC_VAL_MASK: u16 = 0xfff;
        const ADC_CAL_CNT_MAX: u16 = 32;
        const ADC_CAL_CHANNEL: u16 = 15;

        const ADC_SAR1_ENCAL_GND_ADDR: u8 = 0x7;
        const ADC_SAR1_ENCAL_GND_ADDR_MSB: u8 = 5;
        const ADC_SAR1_ENCAL_GND_ADDR_LSB: u8 = 5;

        const ADC_SAR1_DREF_ADDR: u8 = 0x2;
        const ADC_SAR1_DREF_ADDR_MSB: u8 = 0x6;
        const ADC_SAR1_DREF_ADDR_LSB: u8 = 0x4;

        const ADC_SARADC1_ENCAL_REF_ADDR: u8 = 0x7;
        const ADC_SARADC1_ENCAL_REF_ADDR_MSB: u8 = 4;
        const ADC_SARADC1_ENCAL_REF_ADDR_LSB: u8 = 4;

        const ADC_SAR2_ENCAL_GND_ADDR: u8 = 0x7;
        const ADC_SAR2_ENCAL_GND_ADDR_MSB: u8 = 5;
        const ADC_SAR2_ENCAL_GND_ADDR_LSB: u8 = 5;

        const ADC_SAR2_DREF_ADDR: u8 = 0x5;
        const ADC_SAR2_DREF_ADDR_MSB: u8 = 0x6;
        const ADC_SAR2_DREF_ADDR_LSB: u8 = 0x4;

        const ADC_SARADC2_ENCAL_REF_ADDR: u8 = 0x7;
        const ADC_SARADC2_ENCAL_REF_ADDR_MSB: u8 = 4;
        const ADC_SARADC2_ENCAL_REF_ADDR_LSB: u8 = 4;
    }
}

/// The sampling/readout resolution of the ADC.
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Resolution {
    Resolution13Bit,
}

/// An I/O pin which can be read using the ADC.
pub struct AdcPin<PIN, ADCI, CS = ()> {
    pub pin: PIN,
    pub cal_scheme: CS,
    _phantom: PhantomData<ADCI>,
}

impl<PIN, ADCI, CS> embedded_hal::adc::Channel<ADCI> for AdcPin<PIN, ADCI, CS>
where
    PIN: embedded_hal::adc::Channel<ADCI, ID = u8>,
{
    type ID = u8;

    fn channel() -> Self::ID {
        PIN::channel()
    }
}

/// Configuration for the ADC.
pub struct AdcConfig<ADCI> {
    pub resolution: Resolution,
    pub attenuations: [Option<Attenuation>; 10],
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
        PIN: AdcChannel,
    {
        self.attenuations[PIN::CHANNEL as usize] = Some(attenuation);

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
        PIN: AdcChannel,
        CS: AdcCalScheme<ADCI>,
    {
        self.attenuations[PIN::CHANNEL as usize] = Some(attenuation);

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
        ADCI::adc_samplecfg(ADCI::ADC_CAL_CHANNEL);
        ADCI::set_attenuation(ADCI::ADC_CAL_CHANNEL as usize, atten as u8);

        // Connect calibration source
        ADCI::connect_cal(source, true);

        for _ in 0..ADCI::ADC_CAL_CNT_MAX {
            ADCI::set_init_code(0);

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

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        AdcConfig {
            resolution: Resolution::Resolution13Bit,
            attenuations: [None; 10],
            _phantom: PhantomData::default(),
        }
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    fn adc_samplecfg(channel: u16);

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

#[doc(hidden)]
pub trait CalibrationAccess: RegisterAccess {
    const ADC_CAL_CNT_MAX: u16;
    const ADC_CAL_CHANNEL: u16;
    const ADC_VAL_MASK: u16;

    fn enable_vdef(enable: bool);

    /// Enable internal calibration voltage source
    fn connect_cal(source: AdcCalSource, enable: bool);
}

impl RegisterAccess for crate::peripherals::ADC1 {
    fn adc_samplecfg(channel: u16) {
        let sensors = unsafe { &*SENS::ptr() };

        // Configure for RTC control
        sensors.sar_meas1_mux().modify(|_r, w| {
            w.sar1_dig_force().clear_bit() // 1: Select digital control;
                                           // 0: Select RTC control.
        });
        sensors.sar_meas1_ctrl2().modify(|_r, w| {
            w.meas1_start_force()
                .set_bit() // 1: SW control RTC ADC start;     0: ULP control RTC ADC start.
                .sar1_en_pad_force()
                .set_bit() // 1: SW control RTC ADC bit map;   0: ULP control RTC ADC bit map;
                // Enable internal connect GND (for calibration).
                .sar1_en_pad()
                .variant(channel) // only one channel is selected.
        });
    }

    fn set_attenuation(channel: usize, attenuation: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_atten1().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation as u8 & 0b11) as u32) << (channel * 2));

            unsafe { w.sar1_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_mux()
            .modify(|_, w| w.sar1_dig_force().clear_bit());
    }

    fn set_start_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_force().set_bit());
    }

    fn set_en_pad_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| w.sar1_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| unsafe { w.sar1_en_pad().bits(1 << channel) });
    }

    fn clear_start_sample() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }

    fn start_sample() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().set_bit());
    }

    fn is_done() -> bool {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2()
            .read()
            .meas1_done_sar()
            .bit_is_set()
    }

    fn read_data() -> u16 {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas1_ctrl2().read().meas1_data_sar().bits() as u16
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_INITIAL_CODE_HIGH_ADDR, msb as u32);
        crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_INITIAL_CODE_LOW_ADDR, lsb as u32);
    }

    fn reset() {
        let adc = unsafe { &*APB_SARADC::ptr() };
        let sensors = unsafe { &*SENS::ptr() };

        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                adc.int_clr()
                    .write(|w| w.adc1_done_int_clr().set_bit());
            } else {
                adc.int_clr()
                    .write(|w| w.apb_saradc1_done_int_clr().set_bit());
            }
        }

        sensors
            .sar_meas1_ctrl2()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }
}

#[cfg(esp32s3)]
impl CalibrationAccess for crate::peripherals::ADC1 {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_DREF_ADDR, enable as u8);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_ENCAL_GND_ADDR, enable as u8);
            }
            AdcCalSource::Ref => {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC1_ENCAL_REF_ADDR, enable as u8);
            }
        }
    }
}

impl RegisterAccess for crate::peripherals::ADC2 {
    fn adc_samplecfg(channel: u16) {
        let sensors = unsafe { &*SENS::ptr() };

        // Configure for RTC control
        sensors.sar_meas2_ctrl2().modify(|_r, w| {
            w.meas2_start_force()
                .set_bit() // 1: SW control RTC ADC start;     0: ULP control RTC ADC start.
                .sar2_en_pad_force()
                .set_bit() // 1: SW control RTC ADC bit map;   0: ULP control RTC ADC bit map;
                // Enable internal connect GND (for calibration).
                .sar2_en_pad()
                .variant(channel) // only one channel is selected.
        });
    }

    fn set_attenuation(channel: usize, attenuation: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_atten2().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation as u8 & 0b11) as u32) << (channel * 2));

            unsafe { w.sar2_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_mux()
            .modify(|_, w| w.sar2_rtc_force().set_bit());

        let sar_apb = unsafe { &*APB_SARADC::ptr() };
        sar_apb
            .arb_ctrl()
            .modify(|_, w| w.adc_arb_rtc_force().set_bit());
    }

    fn set_start_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_force().set_bit());
    }

    fn set_en_pad_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| w.sar2_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| unsafe { w.sar2_en_pad().bits(1 << channel) });
    }

    fn clear_start_sample() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }

    fn start_sample() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().set_bit());
    }

    fn is_done() -> bool {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2()
            .read()
            .meas2_done_sar()
            .bit_is_set()
    }

    fn read_data() -> u16 {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas2_ctrl2().read().meas2_data_sar().bits() as u16
    }

    fn set_init_code(data: u16) {
        let [msb, lsb] = data.to_be_bytes();

        crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_INITIAL_CODE_HIGH_ADDR, msb as u32);
        crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_INITIAL_CODE_LOW_ADDR, lsb as u32);
    }

    fn reset() {
        let adc = unsafe { &*APB_SARADC::ptr() };
        let sensors = unsafe { &*SENS::ptr() };

        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                adc.int_clr()
                    .write(|w| w.adc2_done_int_clr().set_bit());
            } else {
                adc.int_clr()
                    .write(|w| w.apb_saradc2_done_int_clr().set_bit());
            }
        }

        sensors
            .sar_meas2_ctrl2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }
}

#[cfg(esp32s3)]
impl CalibrationAccess for crate::peripherals::ADC2 {
    const ADC_CAL_CNT_MAX: u16 = ADC_CAL_CNT_MAX;
    const ADC_CAL_CHANNEL: u16 = ADC_CAL_CHANNEL;
    const ADC_VAL_MASK: u16 = ADC_VAL_MASK;

    fn enable_vdef(enable: bool) {
        crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_DREF_ADDR, enable as u8);
    }

    fn connect_cal(source: AdcCalSource, enable: bool) {
        match source {
            AdcCalSource::Gnd => {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_ENCAL_GND_ADDR, enable as u8);
            }
            AdcCalSource::Ref => {
                crate::regi2c_write_mask!(I2C_SAR_ADC, ADC_SARADC2_ENCAL_REF_ADDR, enable as u8);
            }
        }
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct ADC<'d, ADC> {
    _adc: PeripheralRef<'d, ADC>,
    attenuations: [Option<Attenuation>; 10],
    active_channel: Option<u8>,
}

impl<'d, ADCI> ADC<'d, ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn new(
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Self {
        let sensors = unsafe { &*SENS::ptr() };

        // Set attenuation for pins
        let attenuations = config.attenuations;

        for channel in 0..attenuations.len() {
            if let Some(attenuation) = attenuations[channel] {
                ADCI::set_attenuation(channel, attenuation as u8);
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

        ADC {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
        }
    }
}

#[cfg(esp32s3)]
impl AdcCalEfuse for crate::peripherals::ADC1 {
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

#[cfg(esp32s3)]
impl AdcCalEfuse for crate::peripherals::ADC2 {
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

impl<'d, ADCI, PIN, CS> embedded_hal::adc::OneShot<ADCI, u16, AdcPin<PIN, ADCI, CS>>
    for ADC<'d, ADCI>
where
    PIN: embedded_hal::adc::Channel<ADCI, ID = u8>,
    ADCI: RegisterAccess,
    CS: AdcCalScheme<ADCI>,
{
    type Error = ();

    fn read(&mut self, pin: &mut AdcPin<PIN, ADCI, CS>) -> nb::Result<u16, Self::Error> {
        use embedded_hal::adc::Channel;

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

            ADCI::set_en_pad(AdcPin::<PIN, ADCI>::channel() as u8);

            ADCI::clear_start_sample();
            ADCI::start_sample();
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
}

macro_rules! impl_adc_interface {
    ($adc:ident [
        $( ($pin:ident, $channel:expr) ,)+
    ]) => {

        $(
            impl $crate::analog::adc::AdcChannel for crate::gpio::$pin<crate::gpio::Analog> {
                const CHANNEL: u8 = $channel;
            }

            impl embedded_hal::adc::Channel<$adc> for crate::gpio::$pin<crate::gpio::Analog> {
                type ID = u8;

                fn channel() -> u8 { $channel }
            }
        )+
    }
}

mod implementation {
    use crate::peripherals::{ADC1, ADC2};

    impl_adc_interface! {
        ADC1 [
            (Gpio1,  0),
            (Gpio2,  1),
            (Gpio3,  2),
            (Gpio4,  3),
            (Gpio5,  4),
            (Gpio6,  5),
            (Gpio7,  6),
            (Gpio8,  7),
            (Gpio9,  8),
            (Gpio10, 9),
        ]
    }

    impl_adc_interface! {
        ADC2 [
            (Gpio11, 0),
            (Gpio12, 1),
            (Gpio13, 2),
            (Gpio14, 3),
            (Gpio15, 4),
            (Gpio16, 5),
            (Gpio17, 6),
            (Gpio18, 7),
            (Gpio19, 8),
            (Gpio20, 9),
        ]
    }
}

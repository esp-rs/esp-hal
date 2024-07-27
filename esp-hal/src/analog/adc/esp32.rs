use super::{AdcConfig, Attenuation};
use crate::{
    peripheral::PeripheralRef,
    peripherals::{ADC1, ADC2, RTC_IO, SENS},
};

pub(super) const NUM_ATTENS: usize = 10;

/// The sampling/readout resolution of the ADC.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Resolution {
    /// 9-bit resolution
    Resolution9Bit  = 0b00,
    /// 10-bit resolution
    Resolution10Bit = 0b01,
    /// 11-bit resolution
    Resolution11Bit = 0b10,
    /// 12-bit resolution
    #[default]
    Resolution12Bit = 0b11,
}

#[doc(hidden)]
pub trait RegisterAccess {
    fn set_bit_width(resolution: u8);

    fn set_sample_bit(resolution: u8);

    fn set_attenuation(channel: usize, attenuation: u8);

    fn clear_dig_force();

    fn set_start_force();

    fn set_en_pad_force();

    fn set_en_pad(channel: u8);

    fn clear_start_sar();

    fn set_start_sar();

    fn read_done_sar() -> bool;

    fn read_data_sar() -> u16;
}

impl RegisterAccess for ADC1 {
    fn set_bit_width(resolution: u8) {
        unsafe { &*SENS::ptr() }
            .sar_start_force()
            .modify(|_, w| unsafe { w.sar1_bit_width().bits(resolution) });
    }

    fn set_sample_bit(resolution: u8) {
        unsafe { &*SENS::ptr() }
            .sar_read_ctrl()
            .modify(|_, w| unsafe { w.sar1_sample_bit().bits(resolution) });
    }

    fn set_attenuation(channel: usize, attenuation: u8) {
        unsafe { &*SENS::ptr() }.sar_atten1().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation & 0b11) as u32) << (channel * 2));

            unsafe { w.sar1_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        unsafe { &*SENS::ptr() }
            .sar_read_ctrl()
            .modify(|_, w| w.sar1_dig_force().clear_bit());
    }

    fn set_start_force() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .modify(|_, w| w.meas1_start_force().set_bit());
    }

    fn set_en_pad_force() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .modify(|_, w| w.sar1_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .modify(|_, w| unsafe { w.sar1_en_pad().bits(1 << channel) });
    }

    fn clear_start_sar() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }

    fn set_start_sar() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .modify(|_, w| w.meas1_start_sar().set_bit());
    }

    fn read_done_sar() -> bool {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .read()
            .meas1_done_sar()
            .bit_is_set()
    }

    fn read_data_sar() -> u16 {
        unsafe { &*SENS::ptr() }
            .sar_meas_start1()
            .read()
            .meas1_data_sar()
            .bits()
    }
}

impl RegisterAccess for ADC2 {
    fn set_bit_width(resolution: u8) {
        unsafe { &*SENS::ptr() }
            .sar_start_force()
            .modify(|_, w| unsafe { w.sar2_bit_width().bits(resolution) });
    }

    fn set_sample_bit(resolution: u8) {
        unsafe { &*SENS::ptr() }
            .sar_read_ctrl2()
            .modify(|_, w| unsafe { w.sar2_sample_bit().bits(resolution) });
    }

    fn set_attenuation(channel: usize, attenuation: u8) {
        unsafe { &*SENS::ptr() }.sar_atten2().modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation & 0b11) as u32) << (channel * 2));

            unsafe { w.sar2_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        unsafe { &*SENS::ptr() }
            .sar_read_ctrl2()
            .modify(|_, w| w.sar2_dig_force().clear_bit());
    }

    fn set_start_force() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .modify(|_, w| w.meas2_start_force().set_bit());
    }

    fn set_en_pad_force() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .modify(|_, w| w.sar2_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .modify(|_, w| unsafe { w.sar2_en_pad().bits(1 << channel) });
    }

    fn clear_start_sar() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }

    fn set_start_sar() {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .modify(|_, w| w.meas2_start_sar().set_bit());
    }

    fn read_done_sar() -> bool {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .read()
            .meas2_done_sar()
            .bit_is_set()
    }

    fn read_data_sar() -> u16 {
        unsafe { &*SENS::ptr() }
            .sar_meas_start2()
            .read()
            .meas2_data_sar()
            .bits()
    }
}

/// Analog-to-Digital Converter peripheral driver.
pub struct Adc<'d, ADC> {
    _adc: PeripheralRef<'d, ADC>,
    attenuations: [Option<Attenuation>; NUM_ATTENS],
    active_channel: Option<u8>,
}

impl<'d, ADCI> Adc<'d, ADCI>
where
    ADCI: RegisterAccess,
{
    /// Configure a given ADC instance using the provided configuration, and
    /// initialize the ADC for use
    pub fn new(
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Self {
        let sensors = unsafe { &*SENS::ptr() };

        // Set reading and sampling resolution
        let resolution: u8 = config.resolution as u8;

        ADCI::set_bit_width(resolution);
        ADCI::set_sample_bit(resolution);

        // Set attenuation for pins
        let attenuations = config.attenuations;

        for (channel, attentuation) in attenuations.iter().enumerate() {
            if let Some(attenuation) = attentuation {
                ADC1::set_attenuation(channel, *attenuation as u8);
            }
        }

        // Set controller to RTC
        ADCI::clear_dig_force();
        ADCI::set_start_force();
        ADCI::set_en_pad_force();
        sensors
            .sar_touch_ctrl1()
            .modify(|_, w| w.xpd_hall_force().set_bit());
        sensors
            .sar_touch_ctrl1()
            .modify(|_, w| w.hall_phase_force().set_bit());

        // Set power to SW power on
        sensors
            .sar_meas_wait2()
            .modify(|_, w| unsafe { w.force_xpd_sar().bits(0b11) });

        // disable AMP
        sensors
            .sar_meas_wait2()
            .modify(|_, w| unsafe { w.force_xpd_amp().bits(0b10) });
        sensors
            .sar_meas_ctrl()
            .modify(|_, w| unsafe { w.amp_rst_fb_fsm().bits(0) });
        sensors
            .sar_meas_ctrl()
            .modify(|_, w| unsafe { w.amp_short_ref_fsm().bits(0) });
        sensors
            .sar_meas_ctrl()
            .modify(|_, w| unsafe { w.amp_short_ref_gnd_fsm().bits(0) });
        sensors
            .sar_meas_wait1()
            .modify(|_, w| unsafe { w.sar_amp_wait1().bits(1) });
        sensors
            .sar_meas_wait1()
            .modify(|_, w| unsafe { w.sar_amp_wait2().bits(1) });
        sensors
            .sar_meas_wait2()
            .modify(|_, w| unsafe { w.sar_amp_wait3().bits(1) });

        // Do *not* invert the output
        // NOTE: This seems backwards, but was verified experimentally.
        sensors
            .sar_read_ctrl2()
            .modify(|_, w| w.sar2_data_inv().set_bit());

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
    pub fn read_oneshot<PIN>(&mut self, _pin: &mut super::AdcPin<PIN, ADCI>) -> nb::Result<u16, ()>
    where
        PIN: super::AdcChannel,
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

            ADCI::set_en_pad(PIN::CHANNEL);

            ADCI::clear_start_sar();
            ADCI::set_start_sar();
        }

        // Wait for ADC to finish conversion
        let conversion_finished = ADCI::read_done_sar();
        if !conversion_finished {
            return Err(nb::Error::WouldBlock);
        }

        // Get converted value
        let converted_value = ADCI::read_data_sar();

        // Mark that no conversions are currently in progress
        self.active_channel = None;

        Ok(converted_value)
    }
}

impl<'d, ADC1> Adc<'d, ADC1> {
    /// Enable the Hall sensor
    pub fn enable_hall_sensor() {
        unsafe { &*RTC_IO::ptr() }
            .hall_sens()
            .modify(|_, w| w.xpd_hall().set_bit());
    }

    /// Disable the Hall sensor
    pub fn disable_hall_sensor() {
        unsafe { &*RTC_IO::ptr() }
            .hall_sens()
            .modify(|_, w| w.xpd_hall().clear_bit());
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<'d, ADCI, PIN> embedded_hal_02::adc::OneShot<ADCI, u16, super::AdcPin<PIN, ADCI>>
    for Adc<'d, ADCI>
where
    PIN: embedded_hal_02::adc::Channel<ADCI, ID = u8> + super::AdcChannel,
    ADCI: RegisterAccess,
{
    type Error = ();

    fn read(&mut self, pin: &mut super::AdcPin<PIN, ADCI>) -> nb::Result<u16, Self::Error> {
        self.read_oneshot(pin)
    }
}

mod adc_implementation {
    crate::analog::adc::impl_adc_interface! {
        ADC1 [
            (Gpio36, 0), // Alt. name: SENSOR_VP
            (Gpio37, 1), // Alt. name: SENSOR_CAPP
            (Gpio38, 2), // Alt. name: SENSOR_CAPN
            (Gpio39, 3), // Alt. name: SENSOR_VN
            (Gpio33, 4), // Alt. name: 32K_XP
            (Gpio32, 5), // Alt. name: 32K_XN
            (Gpio34, 6), // Alt. name: VDET_1
            (Gpio35, 7), // Alt. name: VDET_2
        ]
    }

    crate::analog::adc::impl_adc_interface! {
        ADC2 [
            (Gpio4,  0),
            (Gpio0,  1),
            (Gpio2,  2),
            (Gpio15, 3), // Alt. name: MTDO
            (Gpio13, 4), // Alt. name: MTCK
            (Gpio12, 5), // Alt. name: MTDI
            (Gpio14, 6), // Alt. name: MTMS
            (Gpio27, 7),
            (Gpio25, 8),
            (Gpio26, 9),
        ]
    }
}

use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};

use crate::{
    analog::{ADC1, ADC2},
    peripheral::PeripheralRef,
    peripherals::{RTC_IO, SENS},
};

/// The sampling/readout resolution of the ADC
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Resolution {
    Resolution9Bit  = 0b00,
    Resolution10Bit = 0b01,
    Resolution11Bit = 0b10,
    Resolution12Bit = 0b11,
}

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
    pub attenuations: [Option<Attenuation>; 10],
    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcConfig<ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn new() -> AdcConfig<ADCI> {
        crate::into_ref!();
        Self::default()
    }

    pub fn enable_pin<PIN: Channel<ADCI, ID = u8>>(
        &mut self,
        pin: PIN,
        attenuation: Attenuation,
    ) -> AdcPin<PIN, ADCI> {
        self.attenuations[PIN::channel() as usize] = Some(attenuation);

        AdcPin {
            pin,
            _phantom: PhantomData::default(),
        }
    }
}

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        AdcConfig {
            resolution: Resolution::Resolution12Bit,
            attenuations: [None; 10],
            _phantom: PhantomData::default(),
        }
    }
}

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

#[doc(hidden)]
impl RegisterAccess for ADC1 {
    fn set_bit_width(resolution: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_start_force
            .modify(|_, w| unsafe { w.sar1_bit_width().bits(resolution) });
    }

    fn set_sample_bit(resolution: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_read_ctrl
            .modify(|_, w| unsafe { w.sar1_sample_bit().bits(resolution) });
    }

    fn set_attenuation(channel: usize, attenuation: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_atten1.modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation as u8 & 0b11) as u32) << (channel * 2));

            unsafe { w.sar1_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_read_ctrl
            .modify(|_, w| w.sar1_dig_force().clear_bit());
    }

    fn set_start_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start1
            .modify(|_, w| w.meas1_start_force().set_bit());
    }

    fn set_en_pad_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start1
            .modify(|_, w| w.sar1_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start1
            .modify(|_, w| unsafe { w.sar1_en_pad().bits(1 << channel) });
    }

    fn clear_start_sar() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start1
            .modify(|_, w| w.meas1_start_sar().clear_bit());
    }

    fn set_start_sar() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start1
            .modify(|_, w| w.meas1_start_sar().set_bit());
    }

    fn read_done_sar() -> bool {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas_start1.read().meas1_done_sar().bit_is_set()
    }

    fn read_data_sar() -> u16 {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas_start1.read().meas1_data_sar().bits() as u16
    }
}

impl RegisterAccess for ADC2 {
    fn set_bit_width(resolution: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_start_force
            .modify(|_, w| unsafe { w.sar2_bit_width().bits(resolution) });
    }

    fn set_sample_bit(resolution: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_read_ctrl2
            .modify(|_, w| unsafe { w.sar2_sample_bit().bits(resolution) });
    }

    fn set_attenuation(channel: usize, attenuation: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_atten2.modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation as u8 & 0b11) as u32) << (channel * 2));

            unsafe { w.sar2_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_read_ctrl2
            .modify(|_, w| w.sar2_dig_force().clear_bit());
    }

    fn set_start_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start2
            .modify(|_, w| w.meas2_start_force().set_bit());
    }

    fn set_en_pad_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start2
            .modify(|_, w| w.sar2_en_pad_force().set_bit());
    }

    fn set_en_pad(channel: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start2
            .modify(|_, w| unsafe { w.sar2_en_pad().bits(1 << channel) });
    }

    fn clear_start_sar() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start2
            .modify(|_, w| w.meas2_start_sar().clear_bit());
    }

    fn set_start_sar() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas_start2
            .modify(|_, w| w.meas2_start_sar().set_bit());
    }

    fn read_done_sar() -> bool {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas_start2.read().meas2_done_sar().bit_is_set()
    }

    fn read_data_sar() -> u16 {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas_start2.read().meas2_data_sar().bits() as u16
    }
}

pub struct ADC<'d, ADC> {
    _adc: PeripheralRef<'d, ADC>,
    attenuations: [Option<Attenuation>; 10],
    active_channel: Option<u8>,
}

impl<'d, ADCI> ADC<'d, ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn adc(
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Result<Self, ()> {
        let sensors = unsafe { &*SENS::ptr() };

        // Set reading and sampling resolution
        let resolution: u8 = config.resolution as u8;

        ADCI::set_bit_width(resolution);
        ADCI::set_sample_bit(resolution);

        // Set attenuation for pins
        let attenuations = config.attenuations;

        for channel in 0..attenuations.len() {
            if let Some(attenuation) = attenuations[channel] {
                ADC1::set_attenuation(channel, attenuation as u8);
            }
        }

        // Set controller to RTC
        ADCI::clear_dig_force();
        ADCI::set_start_force();
        ADCI::set_en_pad_force();
        sensors
            .sar_touch_ctrl1
            .modify(|_, w| w.xpd_hall_force().set_bit());
        sensors
            .sar_touch_ctrl1
            .modify(|_, w| w.hall_phase_force().set_bit());

        // Set power to SW power on
        sensors
            .sar_meas_wait2
            .modify(|_, w| unsafe { w.force_xpd_sar().bits(0b11) });

        // disable AMP
        sensors
            .sar_meas_wait2
            .modify(|_, w| unsafe { w.force_xpd_amp().bits(0b10) });
        sensors
            .sar_meas_ctrl
            .modify(|_, w| unsafe { w.amp_rst_fb_fsm().bits(0) });
        sensors
            .sar_meas_ctrl
            .modify(|_, w| unsafe { w.amp_short_ref_fsm().bits(0) });
        sensors
            .sar_meas_ctrl
            .modify(|_, w| unsafe { w.amp_short_ref_gnd_fsm().bits(0) });
        sensors
            .sar_meas_wait1
            .modify(|_, w| unsafe { w.sar_amp_wait1().bits(1) });
        sensors
            .sar_meas_wait1
            .modify(|_, w| unsafe { w.sar_amp_wait2().bits(1) });
        sensors
            .sar_meas_wait2
            .modify(|_, w| unsafe { w.sar_amp_wait3().bits(1) });

        let adc = ADC {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
        };

        Ok(adc)
    }
}

impl<'d, ADC1> ADC<'d, ADC1> {
    pub fn enable_hall_sensor() {
        // Connect hall sensor
        let rtcio = unsafe { &*RTC_IO::ptr() };
        rtcio.hall_sens.modify(|_, w| w.xpd_hall().set_bit());
    }

    pub fn disable_hall_sensor() {
        // Disconnect hall sensor
        let rtcio = unsafe { &*RTC_IO::ptr() };
        rtcio.hall_sens.modify(|_, w| w.xpd_hall().clear_bit());
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

            ADCI::set_en_pad(AdcPin::<PIN, ADCI>::channel() as u8);

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

pub mod implementation {
    //! Analog to digital (ADC) conversion support.
    //!
    //! This module provides functions for reading analog values from two
    //! analog to digital converters available on the ESP32: `ADC1` and `ADC2`.
    //!
    //! The following pins can be configured for analog readout:
    //!
    //! | Channel | ADC1                 | ADC2          |
    //! |---------|----------------------|---------------|
    //! | 0       | GPIO36 (SENSOR_VP)   | GPIO4         |
    //! | 1       | GPIO37 (SENSOR_CAPP) | GPIO0         |
    //! | 2       | GPIO38 (SENSOR_CAPN) | GPIO2         |
    //! | 3       | GPIO39 (SENSOR_VN)   | GPIO15 (MTDO) |
    //! | 4       | GPIO33 (32K_XP)      | GPIO13 (MTCK) |
    //! | 5       | GPIO32 (32K_XN)      | GPIO12 (MTDI) |
    //! | 6       | GPIO34 (VDET_1)      | GPIO14 (MTMS) |
    //! | 7       | GPIO35 (VDET_2)      | GPIO27        |
    //! | 8       |                      | GPIO25        |
    //! | 9       |                      | GPIO26        |

    use embedded_hal::adc::Channel;

    use super::impl_adc_interface;
    pub use crate::analog::{adc::*, ADC1, ADC2};
    use crate::gpio::*;

    impl_adc_interface! {
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

    impl_adc_interface! {
        ADC2 [
            (Gpio4, 0),
            (Gpio0, 1),
            (Gpio2, 2),
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

use core::marker::PhantomData;

use super::{AdcChannel, Attenuation};
use crate::{
    peripheral::PeripheralRef,
    peripherals::{ADC1, ADC2, RTC_IO, SENS},
};

/// The sampling/readout resolution of the ADC.
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Resolution {
    Resolution9Bit  = 0b00,
    Resolution10Bit = 0b01,
    Resolution11Bit = 0b10,
    Resolution12Bit = 0b11,
}

/// An I/O pin which can be read using the ADC.
pub struct AdcPin<PIN, ADCI> {
    pub pin: PIN,
    _phantom: PhantomData<ADCI>,
}

impl<PIN, ADCI> embedded_hal::adc::Channel<ADCI> for AdcPin<PIN, ADCI>
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
            _phantom: PhantomData,
        }
    }
}

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        AdcConfig {
            resolution: Resolution::Resolution12Bit,
            attenuations: [None; 10],
            _phantom: PhantomData,
        }
    }
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

        ADC {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
        }
    }
}

impl<'d, ADC1> ADC<'d, ADC1> {
    pub fn enable_hall_sensor() {
        // Connect hall sensor
        unsafe { &*RTC_IO::ptr() }
            .hall_sens()
            .modify(|_, w| w.xpd_hall().set_bit());
    }

    pub fn disable_hall_sensor() {
        // Disconnect hall sensor
        unsafe { &*RTC_IO::ptr() }
            .hall_sens()
            .modify(|_, w| w.xpd_hall().clear_bit());
    }
}

impl<'d, ADCI, PIN> embedded_hal::adc::OneShot<ADCI, u16, AdcPin<PIN, ADCI>> for ADC<'d, ADCI>
where
    PIN: embedded_hal::adc::Channel<ADCI, ID = u8>,
    ADCI: RegisterAccess,
{
    type Error = ();

    fn read(&mut self, _pin: &mut AdcPin<PIN, ADCI>) -> nb::Result<u16, Self::Error> {
        use embedded_hal::adc::Channel;

        if self.attenuations[AdcPin::<PIN, ADCI>::channel() as usize].is_none() {
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

            ADCI::set_en_pad(AdcPin::<PIN, ADCI>::channel());

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

mod adc_implementation {
    use crate::peripherals::{ADC1, ADC2};

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

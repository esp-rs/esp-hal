use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};

use crate::{
    analog::{ADC1, ADC2},
    pac::APB_SARADC,
    system::{Peripheral, PeripheralClockControl},
};

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
        _pin: &PIN,
        attenuation: Attenuation,
    ) {
        self.attenuations[PIN::channel() as usize] = Some(attenuation);
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

pub struct ADC<ADC> {
    adc: PhantomData<ADC>,
    attenuations: [Option<Attenuation>; 5],
    active_channel: Option<u8>,
}

impl<ADCI> ADC<ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn adc(
        peripheral_clock_controller: &mut PeripheralClockControl,
        _adc_instance: ADCI,
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
            adc: PhantomData,
            attenuations: config.attenuations,
            active_channel: None,
        };

        Ok(adc)
    }
}

impl<ADCI, WORD, PIN> OneShot<ADCI, WORD, PIN> for ADC<ADCI>
where
    WORD: From<u16>,
    PIN: Channel<ADCI, ID = u8>,
    ADCI: RegisterAccess,
{
    type Error = ();

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        if self.attenuations[PIN::channel() as usize] == None {
            panic!("Channel {} is not configured reading!", PIN::channel());
        }

        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahaid and check progress
            if active_channel != PIN::channel() {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(PIN::channel());

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

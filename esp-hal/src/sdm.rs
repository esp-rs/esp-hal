//! # Sigma-Delta modulation (SDM)
//!
//! ## Overview
//!
//! Almost all ESP SoCs has a second-order sigma-delta modulator, which
//! can generate independent PDM pulses to multiple channels. Please refer
//! to the TRM to check how many hardware channels are available.
//!
//! Delta-sigma modulation converts an analog voltage signal into a pulse
//! frequency, or pulse density, which can be understood as pulse-density
//! modulation (PDM) (refer to [Delta-sigma modulation on Wikipedia](https://en.wikipedia.org/wiki/Delta-sigma_modulation)).
//!
//! Typically, a Sigma-Delta modulated channel can be used in scenarios like:
//!
//! - LED dimming
//! - Simple DAC (8-bit), with the help of an active RC low-pass filter
//! - Class D amplifier, with the help of a half-bridge or full-bridge circuit
//!   plus an LC low-pass filter
//!
//! ## Configuration
//!
//! After creating [`Sdm`] instance you should connect individual channels to
//! GPIO outputs. Also you need set modulation frequency.
//!
//! ## Usage
//!
//! Connected channels accepts pulse density in range -128..127.

use core::marker::PhantomData;

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    gpio::{OutputPin, OutputSignal},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::GPIO_SD,
    private,
};

/// Sigma-Delta modulation peripheral driver.
pub struct Sdm<'d> {
    /// Channel 0
    pub channel0: ChannelRef<'d, 0>,

    /// Channel 1
    pub channel1: ChannelRef<'d, 1>,

    /// Channel 2
    pub channel2: ChannelRef<'d, 2>,

    /// Channel 3
    pub channel3: ChannelRef<'d, 3>,

    #[cfg(any(esp32, esp32s2, esp32s3))]
    /// Channel 4
    pub channel4: ChannelRef<'d, 4>,

    #[cfg(any(esp32, esp32s2, esp32s3))]
    /// Channel 5
    pub channel5: ChannelRef<'d, 5>,

    #[cfg(any(esp32, esp32s2, esp32s3))]
    /// Channel 6
    pub channel6: ChannelRef<'d, 6>,

    #[cfg(any(esp32, esp32s2, esp32s3))]
    /// Channel 7
    pub channel7: ChannelRef<'d, 7>,
}

/// Channel errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Prescale out of range
    PrescaleRange,
}

impl<'d> Drop for Sdm<'d> {
    fn drop(&mut self) {
        GPIO_SD::enable_clock(false);
    }
}

impl<'d> Sdm<'d> {
    /// Initialize driver using a given SD instance.
    pub fn new(_sd: impl crate::peripheral::Peripheral<P = GPIO_SD> + 'd) -> Self {
        GPIO_SD::enable_clock(true);

        Self {
            channel0: ChannelRef::new(),
            channel1: ChannelRef::new(),
            channel2: ChannelRef::new(),
            channel3: ChannelRef::new(),

            #[cfg(any(esp32, esp32s2, esp32s3))]
            channel4: ChannelRef::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            channel5: ChannelRef::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            channel6: ChannelRef::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            channel7: ChannelRef::new(),
        }
    }
}

/// Sigma-Delta modulation channel reference.
pub struct ChannelRef<'d, const N: u8> {
    _phantom: PhantomData<&'d ()>,
}

impl<'d, const N: u8> ChannelRef<'d, N> {
    fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Configure and connect sigma-delta channel to output
    pub fn connect<O: OutputPin>(
        &'d self,
        output: impl Peripheral<P = O> + 'd,
        frequency: HertzU32,
    ) -> Result<Channel<'d, N, O>, Error> {
        crate::into_ref!(output);

        let signal = CHANNELS[N as usize];

        output.connect_peripheral_to_output(signal, private::Internal);

        let channel = Channel { _ref: self, output };

        channel.set_frequency(frequency)?;

        Ok(channel)
    }
}

/// Sigma-Delta modulation channel handle.
pub struct Channel<'d, const N: u8, O: OutputPin> {
    _ref: &'d ChannelRef<'d, N>,
    output: PeripheralRef<'d, O>,
}

impl<'d, const N: u8, O: OutputPin> Drop for Channel<'d, N, O> {
    fn drop(&mut self) {
        let signal = CHANNELS[N as usize];
        self.output
            .disconnect_from_peripheral_output(signal, private::Internal);
    }
}

impl<'d, const N: u8, O: OutputPin> Channel<'d, N, O> {
    /// Set raw pulse density
    ///
    /// Sigma-delta quantized density of one channel, the value ranges from -128
    /// to 127, recommended range is -90 ~ 90. The waveform is more like a
    /// random one in this range.
    pub fn set_pulse_density(&self, density: i8) {
        GPIO_SD::set_pulse_density(N, density);
    }

    /// Set duty cycle
    pub fn set_duty(&self, duty: u8) {
        let density = duty as i16 - 128;
        self.set_pulse_density(density as i8)
    }

    /// Set raw prescale
    ///
    /// The divider of source clock, ranges from 1 to 256
    pub fn set_prescale(&self, prescale: u16) -> Result<(), Error> {
        if (1..=256).contains(&prescale) {
            GPIO_SD::set_prescale(N, prescale);
            Ok(())
        } else {
            Err(Error::PrescaleRange)
        }
    }

    /// Set prescale using frequency
    pub fn set_frequency(&self, frequency: HertzU32) -> Result<(), Error> {
        let clocks = Clocks::get();
        let clock_frequency = clocks.apb_clock.to_Hz();
        let frequency = frequency.to_Hz();

        let prescale = prescale_from_frequency(clock_frequency, frequency);

        self.set_prescale(prescale)
    }
}

mod ehal1 {
    use embedded_hal::pwm::{Error as PwmError, ErrorKind, ErrorType, SetDutyCycle};

    use super::{Channel, Error, OutputPin};

    impl PwmError for Error {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl<'d, const N: u8, O: OutputPin> ErrorType for Channel<'d, N, O> {
        type Error = Error;
    }

    impl<'d, const N: u8, O: OutputPin> SetDutyCycle for Channel<'d, N, O> {
        fn max_duty_cycle(&self) -> u16 {
            255
        }

        fn set_duty_cycle(&mut self, mut duty: u16) -> Result<(), Self::Error> {
            let max = self.max_duty_cycle();
            duty = if duty > max { max } else { duty };
            self.set_duty(duty as u8);
            Ok(())
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
const CHANNELS: [OutputSignal; 8] = [
    OutputSignal::GPIO_SD0,
    OutputSignal::GPIO_SD1,
    OutputSignal::GPIO_SD2,
    OutputSignal::GPIO_SD3,
    OutputSignal::GPIO_SD4,
    OutputSignal::GPIO_SD5,
    OutputSignal::GPIO_SD6,
    OutputSignal::GPIO_SD7,
];

#[cfg(any(esp32c3, esp32c6, esp32h2))]
const CHANNELS: [OutputSignal; 4] = [
    OutputSignal::GPIO_SD0,
    OutputSignal::GPIO_SD1,
    OutputSignal::GPIO_SD2,
    OutputSignal::GPIO_SD3,
];

#[doc(hidden)]
pub trait RegisterAccess {
    /// Enable/disable sigma/delta clock
    fn enable_clock(en: bool);

    /// Set channel pulse density
    fn set_pulse_density(ch: u8, density: i8);

    /// Set channel clock pre-scale
    fn set_prescale(ch: u8, prescale: u16);
}

impl RegisterAccess for GPIO_SD {
    fn enable_clock(_en: bool) {
        // The clk enable register does not exist on ESP32.
        #[cfg(not(esp32))]
        {
            let sd = unsafe { &*Self::PTR };

            sd.sigmadelta_misc()
                .modify(|_, w| w.function_clk_en().bit(_en));
        }
    }

    fn set_pulse_density(ch: u8, density: i8) {
        let sd = unsafe { &*Self::PTR };

        sd.sigmadelta(ch as _)
            .modify(|_, w| unsafe { w.in_().bits(density as _) });
    }

    fn set_prescale(ch: u8, prescale: u16) {
        let sd = unsafe { &*Self::PTR };

        sd.sigmadelta(ch as _)
            .modify(|_, w| unsafe { w.prescale().bits((prescale - 1) as _) });
    }
}

fn prescale_from_frequency(clk_freq: u32, req_freq: u32) -> u16 {
    let pre = clk_freq / req_freq;
    let err = clk_freq % req_freq;

    // Do the normal rounding and error >= (src/n + src/(n+1)) / 2,
    // then carry the bit
    let pre = if err >= clk_freq / (2 * pre * (pre + 1)) {
        pre + 1
    } else {
        pre
    };

    pre as _
}

//! Sigma-Delta modulation peripheral driver.

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    gpio::{OutputSignal, PeripheralOutput},
    peripheral::{Peripheral, PeripheralRef},
    peripherals,
    private,
};

/// Sigma-Delta modulation peripheral driver.
pub struct Sdm<'d, SD> {
    _sd: PeripheralRef<'d, SD>,
    channels_usage: u8,
}

/// Channel errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Prescale out of range
    PrescaleRange,
    /// No free channels to use
    NoChannels,
}

impl<'d, SD> Sdm<'d, SD>
where
    SD: RegisterAccess,
{
    /// Initialize driver using a given SD instance.
    pub fn new(sd_instance: impl crate::peripheral::Peripheral<P = SD> + 'd) -> Self {
        Self {
            _sd: sd_instance.into_ref(),
            channels_usage: 0,
        }
    }

    /// Configure and acquire channel.
    pub fn enable_pin<PIN: PeripheralOutput>(
        &mut self,
        pin: impl Peripheral<P = PIN> + 'd,
        frequency: HertzU32,
    ) -> Result<Channel<'_, SD, PIN>, Error> {
        crate::into_ref!(pin);

        let chs = self.channels_usage;
        let chidx = self.alloc_channel()?;
        let signal = CHANNELS[chidx as usize];

        if chs == 0 {
            SD::enable_clock(true);
        }

        pin.connect_peripheral_to_output(signal, private::Internal);

        let mut channel = Channel {
            _sdm: self,
            pin,
            chidx,
        };

        channel.set_frequency(frequency)?;

        Ok(channel)
    }

    /// Deconfigure and release channel.
    pub fn disable_pin<PIN: PeripheralOutput>(&mut self, channel: Channel<'d, SD, PIN>) {
        let Channel { mut pin, chidx, .. } = channel;

        let signal = CHANNELS[chidx as usize];

        pin.disconnect_from_peripheral_output(signal, private::Internal);

        self.dealloc_channel(chidx);

        if self.channels_usage == 0 {
            SD::enable_clock(false);
        }
    }

    fn alloc_channel(&mut self) -> Result<u8, Error> {
        let mut usage = self.channels_usage;
        let mut chidx: u8 = 0;
        while usage & 1 != 0 && chidx < CHANNELS.len() as u8 {
            usage >>= 1;
            chidx += 1;
        }
        if chidx < CHANNELS.len() as u8 {
            self.channels_usage |= 1 << chidx;
            Ok(chidx)
        } else {
            Err(Error::NoChannels)
        }
    }

    fn dealloc_channel(&mut self, chidx: u8) {
        self.channels_usage &= !(1 << chidx);
    }
}

/// Sigma-Delta modulation channel handle.
pub struct Channel<'d, SD, PIN> {
    _sdm: &'d Sdm<'d, SD>,
    pin: PeripheralRef<'d, PIN>,
    chidx: u8,
}

impl<'d, SD, PIN> Channel<'d, SD, PIN>
where
    SD: RegisterAccess,
{
    /// Set raw pulse density
    ///
    /// Sigma-delta quantized density of one channel, the value ranges from -128
    /// to 127, recommended range is -90 ~ 90. The waveform is more like a
    /// random one in this range.
    pub fn set_pulse_density(&mut self, density: i8) {
        SD::set_pulse_density(self.chidx, density);
    }

    /// Set duty cycle
    pub fn set_duty(&mut self, duty: u8) {
        let density = duty as i16 - 128;
        self.set_pulse_density(density as i8)
    }

    /// Set raw prescale
    ///
    /// The divider of source clock, ranges from 1 to 256
    pub fn set_prescale(&mut self, prescale: u16) -> Result<(), Error> {
        if (1..=256).contains(&prescale) {
            SD::set_prescale(self.chidx, prescale);
            Ok(())
        } else {
            Err(Error::PrescaleRange)
        }
    }

    /// Set prescale using frequency
    pub fn set_frequency(&mut self, frequency: HertzU32) -> Result<(), Error> {
        let clocks = Clocks::get();
        let clock_frequency = clocks.apb_clock.to_Hz();
        let frequency = frequency.to_Hz();

        let prescale = prescale_from_frequency(clock_frequency, frequency);

        self.set_prescale(prescale)
    }
}

mod ehal1 {
    use embedded_hal::pwm::{Error as PwmError, ErrorKind, ErrorType, SetDutyCycle};

    use super::{Channel, Error, RegisterAccess};
    use crate::gpio::OutputPin;

    impl PwmError for Error {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl<'d, SD: RegisterAccess, PIN: OutputPin> ErrorType for Channel<'d, SD, PIN> {
        type Error = Error;
    }

    impl<'d, SD: RegisterAccess, PIN: OutputPin> SetDutyCycle for Channel<'d, SD, PIN> {
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

impl RegisterAccess for peripherals::GPIO_SD {
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

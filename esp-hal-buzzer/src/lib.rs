//! # Buzzer
//!
//! ## Overview
//! This driver provides an abstraction over LEDC to drive a piezo-electric
//! buzzer through a user-friendly API.
//!
//! The [songs] module contains pre-programmed songs to play through the buzzer.
//! ## Example
//!
//! ```rust,ignore
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! let mut ledc = Ledc::new(peripherals.LEDC, &clocks);
//! ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
//!
//! let mut buzzer = Buzzer::new(
//!     &ledc,
//!     timer::Number::Timer0,
//!     channel::Number::Channel1,
//!     io.pins.gpio6,
//!     &clocks,
//! );
//!
//! // Play a 1000Hz frequency
//! buzzer.play(1000).unwrap()
//! ```
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![deny(missing_docs)]
#![no_std]

use core::{fmt::Debug, ops::DerefMut};

use esp_hal::{
    clock::Clocks,
    delay::Delay,
    gpio::{AnyPin, CreateErasedPin, InputPin, Level, Output, OutputPin},
    ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::{self, Timer, TimerHW, TimerIFace, TimerSpeed},
        Ledc,
        LowSpeed,
    },
    peripheral::{Peripheral, PeripheralRef},
};
use fugit::RateExtU32;

pub mod notes;

/// Errors from Buzzer
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Errors from [channel::Error]
    Channel(channel::Error),

    /// Errors from [timer::Error]
    Timer(timer::Error),

    /// Error when the volume pin isn't set and we try to use it
    VolumeNotSet,

    /// When the volume level is out of range. Either too low or too high.
    VolumeOutOfRange,
}

/// Converts [channel::Error] into [self::Error]
impl From<channel::Error> for Error {
    fn from(error: channel::Error) -> Self {
        Error::Channel(error)
    }
}

/// Converts [timer::Error] into [self::Error]
impl From<timer::Error> for Error {
    fn from(error: timer::Error) -> Self {
        Error::Timer(error)
    }
}

/// Represents a tone value to play through the buzzer
pub struct ToneValue {
    /// Frequency of the tone in Hz  
    /// *Use 0 for a silent tone*
    pub frequency: u32,

    /// Duration for the frequency in ms
    pub duration: u32,
}

/// Represents different volume strategies for the buzzer.
///
/// - [VolumeType::OnOff] is a simple on or off volume. It's similar as using
///   `.mute()` except that the volume control is on a second pin independent of
///   the buzzer.
///
/// - [VolumeType::Duty] uses the duty as the volume control. It acts like a PWM
///   by switching the power on and off. This may require extra logic gates in
///   the circuit.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VolumeType {
    /// An On / Off based volume
    OnOff,

    /// A duty based volume where 0% is the lowest and 100% the highest.
    Duty,
}

/// Volume configuration for the buzzer
struct Volume<'a> {
    /// Output pin for the volume
    volume_pin: AnyPin<'a>,

    /// Type of the volume
    volume_type: VolumeType,

    /// Volume level
    ///
    /// For [VolumeType::OnOff], should be 0 for Off, or 1 or more for On
    /// For [VolumeType::Duty], should be between 0 and 100.
    level: u8,
}

/// A buzzer instance driven by Ledc
pub struct Buzzer<'a, S: TimerSpeed, O: OutputPin> {
    timer: Timer<'a, S>,
    channel_number: channel::Number,
    output_pin: PeripheralRef<'a, O>,
    delay: Delay,
    volume: Option<Volume<'a>>,
    clocks: &'a Clocks<'a>,
}

impl<'a, S: TimerSpeed, O: OutputPin + Peripheral<P = O>> Buzzer<'a, S, O>
where
    S: TimerSpeed<ClockSourceType = timer::LSClockSource>,
    Timer<'a, S>: TimerHW<S>,
    Timer<'a, S>: TimerIFace<LowSpeed>,
{
    /// Create a new buzzer for the given pin
    pub fn new(
        ledc: &'a Ledc,
        timer_number: timer::Number,
        channel_number: channel::Number,
        output_pin: impl Peripheral<P = O> + 'a,
        clocks: &'a Clocks,
    ) -> Self {
        let timer = ledc.get_timer(timer_number);
        Self {
            timer,
            channel_number,
            output_pin: output_pin.into_ref(),
            delay: Delay::new(clocks),
            volume: None::<Volume<'a>>,
            clocks,
        }
    }

    /// Add a volume control for the buzzer.
    pub fn with_volume<V: OutputPin + InputPin + CreateErasedPin>(
        mut self,
        volume_pin: impl Peripheral<P = V> + 'a,
        volume_type: VolumeType,
    ) -> Self {
        self.volume = Some(Volume {
            volume_pin: AnyPin::new(volume_pin),
            volume_type,
            level: 50,
        });

        self
    }

    /// Set the volume of the buzzer
    ///
    /// For [VolumeType::Duty], the level should be between 0 and 100.
    /// For [VolumeType::OnOff], it will only be mute on 0 and playing on 1 or
    /// more
    pub fn set_volume(&mut self, level: u8) -> Result<(), Error> {
        if let Some(ref mut volume) = self.volume {
            match volume.volume_type {
                VolumeType::OnOff => {
                    // Only turn off when level is set to 0, else set to high
                    Output::new(
                        unsafe { volume.volume_pin.clone_unchecked() },
                        if level != 0 { Level::High } else { Level::Low },
                    );
                    Ok(())
                }
                VolumeType::Duty => {
                    match level {
                        0..=99 => {
                            volume.level = level;

                            // Put a dummy config in the timer if it's not already configured
                            if !self.timer.is_configured() {
                                self.timer.configure(timer::config::Config {
                                    duty: timer::config::Duty::Duty11Bit,
                                    clock_source: timer::LSClockSource::APBClk,
                                    frequency: 20_000.Hz(),
                                })?;
                            }

                            let mut channel = Channel::new(self.channel_number, unsafe {
                                volume.volume_pin.clone_unchecked()
                            });
                            channel
                                .configure(channel::config::Config {
                                    timer: &self.timer,
                                    duty_pct: level,
                                    pin_config: channel::config::PinConfig::PushPull,
                                })
                                .map_err(|e| e.into())
                        }
                        100 => {
                            // If level is 100, we just keep the pin high
                            Output::new(
                                unsafe { volume.volume_pin.clone_unchecked() },
                                Level::High,
                            );
                            Ok(())
                        }
                        _ => Err(Error::VolumeOutOfRange),
                    }
                }
            }
        } else {
            Err(Error::VolumeNotSet)
        }
    }

    /// Mute the buzzer
    ///
    /// The muting is done by simply setting the duty to 0
    pub fn mute(&mut self) -> Result<(), Error> {
        let mut channel = Channel::new(self.channel_number, self.output_pin.deref_mut());
        channel
            .configure(channel::config::Config {
                timer: &self.timer,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .map_err(|e| e.into())
    }

    /// Play a frequency through the buzzer
    pub fn play(&mut self, frequency: u32) -> Result<(), Error> {
        // Mute if frequency is 0Hz
        if frequency == 0 {
            return self.mute();
        }

        // Max duty resolution for a frequency:
        // Integer(log2(LEDC_APB_CKL / frequency))
        let mut result = 0;
        let mut value = (self.clocks.apb_clock / frequency).raw();

        // Limit duty resolution to 14 bits
        while value > 1 && result < 14 {
            value >>= 1;
            result += 1;
        }

        self.timer.configure(timer::config::Config {
            // Safety: This should never fail because resolution is limited to 14 bits
            duty: timer::config::Duty::try_from(result).unwrap(),
            clock_source: timer::LSClockSource::APBClk,
            frequency: frequency.Hz(),
        })?;

        let mut channel = Channel::new(self.channel_number, self.output_pin.deref_mut());
        channel.configure(channel::config::Config {
            timer: &self.timer,
            // Use volume as duty if set since we use the same channel.
            duty_pct: self.volume.as_ref().map_or(50, |v| v.level),
            pin_config: channel::config::PinConfig::PushPull,
        })?;

        Ok(())
    }

    /// Play a sound sequence through the buzzer
    ///
    /// Uses a pair of frequencies and timings to play a sound sequence.
    ///
    /// # Arguments
    /// * `sequence` - A list of frequencies to play through the buzzer
    /// * `timings` - A list of timings in ms for each frequencies
    ///
    /// # Examples
    /// Play a single beep at 300Hz for 1 second
    /// ```
    /// buzzer.play_tones([300], [1000]);
    /// ```
    ///
    /// Play a sequence of 3 beeps with a break inbetween
    /// ```
    /// buzzer.play_tones([200, 0, 200, 0, 200], [200, 50, 200, 50, 200]);
    /// ```
    ///
    /// Play a sequence of 3 beeps with the same duration
    /// ```
    /// buzzer.play_tones([100, 200, 300], [100; 3]);
    /// ```
    ///
    /// # Errors
    /// This function returns an [Error] in case of an error.
    /// An error can occur when an invalid value is used as a tone
    pub fn play_tones<const T: usize>(
        &mut self,
        sequence: [u32; T],
        timings: [u32; T],
    ) -> Result<(), Error> {
        // Iterate for each frequency / timing pair
        for (frequency, timing) in sequence.iter().zip(timings.iter()) {
            self.play(*frequency)?;
            self.delay.delay_millis(*timing);
            self.mute()?;
        }
        // Mute at the end of the sequence
        self.mute()
    }

    /// Play a tone sequence through the buzzer
    ///
    /// Uses a pair of frequencies and timings to play a sound sequence.
    ///
    /// # Arguments
    /// * `tones` - A list of type [ToneValue] to play through the buzzer
    ///
    /// # Examples
    /// Play a tone sequence
    /// ```
    /// let song = [
    ///     ToneValue {
    ///         frequency: 100,
    ///         duration: 100,
    ///     },
    ///     ToneValue {
    ///         frequency: 200,
    ///         duration: 100,
    ///     },
    ///     ToneValue {
    ///         frequency: 300,
    ///         duration: 100,
    ///     },
    /// ];
    /// buzzer.play_song(song);
    /// ```
    ///
    /// # Errors
    /// This function returns an [Error] in case of an error.
    /// An error can occur when an invalid value is used as a tone
    pub fn play_song<const T: usize>(&mut self, tones: [ToneValue; T]) -> Result<(), Error> {
        let mut sequence: [u32; T] = [0; T];
        let mut timings: [u32; T] = [0; T];
        for (index, tone) in tones.iter().enumerate() {
            sequence[index] = tone.frequency;
            timings[index] = tone.duration;
        }
        self.play_tones(sequence, timings)
    }
}

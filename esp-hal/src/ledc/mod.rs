#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # LED Controller (LEDC)
//!
//! ## Overview
//!
//! The LEDC peripheral is primarily designed to control the intensity of LEDs,
//! although it can also be used to generate PWM signals for other purposes. It
//! has multiple channels which can generate independent waveforms that can be
//! used, for example, to drive RGB LED devices.
//!
//! The PWM controller can automatically increase or decrease the duty cycle
//! gradually, allowing for fades without any processor interference.
//!
//! For more information, please refer to the
#![doc = crate::trm_markdown_link!("ledpwm")]
//! ## Configuration
//! Currently only supports fixed-frequency output. High Speed channels are
//! available for the ESP32 only, while Low Speed channels are available for all
//! supported chips.
//!
//! ## Usage
//!
//! The LEDC driver implements the `SetDutyCycle` trait from `embedded-hal` for
//! the `Channel`s.
//!
//! ## Examples
//!
//! ### Low Speed Channel
//!
//! The following example will configure the Low Speed Channel0 to 24kHz output
//! using the APB clock and turn on an LED, then initiate a hardware-controlled
//! fade effect.
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::ledc::{self, Ledc};
//! # use esp_hal::time::Rate;
//!
//! // Create a new Ledc driver and initialize the global slow clock.
//! let mut ledc = Ledc::new(peripherals.LEDC, ledc::Config::default())?;
//!
//! // Initialize a new timer.
//! let timer0 = ledc
//!     .timer0
//!     .configure(ledc::timer::Config::default().with_frequency(Rate::from_khz(24)))?;
//!
//! // Initialize a new channel connected to the timer.
//! // The initial duty cycle is set to 0 (fully off).
//! let mut channel0 = ledc
//!     .channel0
//!     .configure(&timer0, ledc::channel::Config::default())?
//!     .with_pin(peripherals.GPIO0);
//!
//! // Get the maximum duty cycle for the configured timer resolution.
//! let max_duty = channel0.max_duty_cycle();
//!
//! loop {
//!     // Set up a breathing LED: fade from off to on over a second, then
//!     // from on back off over the next second. Then loop.
//!     channel0.start_duty_fade(0, max_duty, 1000)?;
//!     while channel0.is_duty_fade_running() {}
//!     channel0.start_duty_fade(max_duty, 0, 1000)?;
//!     while channel0.is_duty_fade_running() {}
//! }
//! # {after_snippet}
//! ```
//!
//! ## Implementation State
//! - Source clock selection is not supported
//! - Interrupts are not supported

use core::marker::PhantomData;

use crate::{
    Blocking,
    DriverMode,
    ledc::{channel::ChannelCreator, timer::TimerCreator},
    peripherals::LEDC,
    private::Sealed,
    system::{Peripheral, PeripheralGuard},
};

pub mod channel;
mod low_level;
pub mod timer;

/// Global slow clock source
#[instability::unstable]
#[derive(PartialEq, Eq, Copy, Clone, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LowSpeedGlobalClockSource {
    /// APB clock
    APBClock,
}

/// Ledc configuration errors
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, _f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {}
    }
}

impl core::error::Error for ConfigError {}

/// Ledc configuration
#[instability::unstable]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Global slow clock source
    clock_source: LowSpeedGlobalClockSource,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: LowSpeedGlobalClockSource::APBClock,
        }
    }
}

#[cfg(ledc_version = "1")]
/// Used to specify HighSpeed Timer/Channel
#[instability::unstable]
#[derive(PartialEq, Eq, Clone, Copy, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HighSpeed;

/// Used to specify LowSpeed Timer/Channel
#[instability::unstable]
#[derive(PartialEq, Eq, Clone, Copy, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LowSpeed;

/// Trait representing the speed mode of a clock or peripheral
#[instability::unstable]
pub trait Speed: Sealed {
    /// Boolean constant indicating whether the speed is high-speed
    const IS_HS: bool;
}

#[cfg(ledc_version = "1")]
impl Sealed for HighSpeed {}

impl Sealed for LowSpeed {}

#[cfg(ledc_version = "1")]
impl Speed for HighSpeed {
    const IS_HS: bool = true;
}

impl Speed for LowSpeed {
    const IS_HS: bool = false;
}

/// LEDC (LED PWM Controller).
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ledc<'d, Dm: DriverMode> {
    _instance: LEDC<'d>,
    _guard: PeripheralGuard,
    _phantom: PhantomData<Dm>,
    /// Low Speed Timer 0
    pub timer0: TimerCreator<'d, 0, LowSpeed>,
    /// Low Speed Timer 1
    pub timer1: TimerCreator<'d, 1, LowSpeed>,
    /// Low Speed Timer 2
    pub timer2: TimerCreator<'d, 2, LowSpeed>,
    /// Low Speed Timer 3
    pub timer3: TimerCreator<'d, 3, LowSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 0
    pub hs_timer0: TimerCreator<'d, 0, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 1
    pub hs_timer1: TimerCreator<'d, 1, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 2
    pub hs_timer2: TimerCreator<'d, 2, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Timer 3
    pub hs_timer3: TimerCreator<'d, 3, HighSpeed>,

    /// Low Speed Channel 0
    pub channel0: ChannelCreator<'d, 0, Dm, LowSpeed>,
    /// Low Speed Channel 1
    pub channel1: ChannelCreator<'d, 1, Dm, LowSpeed>,
    /// Low Speed Channel 2
    pub channel2: ChannelCreator<'d, 2, Dm, LowSpeed>,
    /// Low Speed Channel 3
    pub channel3: ChannelCreator<'d, 3, Dm, LowSpeed>,
    /// Low Speed Channel 4
    pub channel4: ChannelCreator<'d, 4, Dm, LowSpeed>,
    /// Low Speed Channel 5
    pub channel5: ChannelCreator<'d, 5, Dm, LowSpeed>,
    #[cfg(ledc_channel_count = "8")]
    /// Low Speed Channel 6
    pub channel6: ChannelCreator<'d, 6, Dm, LowSpeed>,
    #[cfg(ledc_channel_count = "8")]
    /// Low Speed Channel 7
    pub channel7: ChannelCreator<'d, 7, Dm, LowSpeed>,

    #[cfg(ledc_version = "1")]
    /// High Speed Channel 0
    pub hs_channel0: ChannelCreator<'d, 0, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 1
    pub hs_channel1: ChannelCreator<'d, 1, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 2
    pub hs_channel2: ChannelCreator<'d, 2, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 3
    pub hs_channel3: ChannelCreator<'d, 3, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 4
    pub hs_channel4: ChannelCreator<'d, 4, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 5
    pub hs_channel5: ChannelCreator<'d, 5, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 6
    pub hs_channel6: ChannelCreator<'d, 6, Dm, HighSpeed>,
    #[cfg(ledc_version = "1")]
    /// High Speed Channel 7
    pub hs_channel7: ChannelCreator<'d, 7, Dm, HighSpeed>,
}

impl<'d> Ledc<'d, Blocking> {
    /// Creates a new `Ledc` instance.
    #[instability::unstable]
    pub fn new(instance: LEDC<'d>, config: Config) -> Result<Self, ConfigError> {
        let guard = PeripheralGuard::new(Peripheral::Ledc);

        let mut ledc = Self {
            _instance: instance,
            _guard: guard,
            _phantom: PhantomData,

            timer0: unsafe { TimerCreator::steal() },
            timer1: unsafe { TimerCreator::steal() },
            timer2: unsafe { TimerCreator::steal() },
            timer3: unsafe { TimerCreator::steal() },

            #[cfg(ledc_version = "1")]
            hs_timer0: unsafe { TimerCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_timer1: unsafe { TimerCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_timer2: unsafe { TimerCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_timer3: unsafe { TimerCreator::steal() },

            channel0: unsafe { ChannelCreator::steal() },
            channel1: unsafe { ChannelCreator::steal() },
            channel2: unsafe { ChannelCreator::steal() },
            channel3: unsafe { ChannelCreator::steal() },
            channel4: unsafe { ChannelCreator::steal() },
            channel5: unsafe { ChannelCreator::steal() },

            #[cfg(ledc_channel_count = "8")]
            channel6: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_channel_count = "8")]
            channel7: unsafe { ChannelCreator::steal() },

            #[cfg(ledc_version = "1")]
            hs_channel0: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel1: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel2: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel3: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel4: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel5: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel6: unsafe { ChannelCreator::steal() },
            #[cfg(ledc_version = "1")]
            hs_channel7: unsafe { ChannelCreator::steal() },
        };
        ledc.apply_config(&config)?;

        Ok(ledc)
    }
}

impl<'d, Dm: DriverMode> Ledc<'d, Dm> {
    /// Changes the configuration.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        low_level::set_global_slow_clock(LEDC::regs(), config.clock_source);
        Ok(())
    }
}

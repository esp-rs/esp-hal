//! # LEDC channel
//!
//! ## Overview
//! The LEDC Channel module provides a high-level interface to
//! configure and control individual PWM channels of the LEDC peripheral.
//!
//! ## Configuration
//! The module allows precise and flexible control over LED lighting and other
//! Pulse-Width Modulation (PWM) applications by offering configurable duty
//! cycles and frequencies.
//!
//! For more information, please refer to the
#![doc = crate::trm_markdown_link!("ledpwm")]

use core::{fmt::Display, marker::PhantomData, sync::atomic::Ordering};

use super::low_level;
use crate::{
    DriverMode,
    gpio::{PinGuard, interconnect::PeripheralOutput},
    ledc::{
        Speed,
        timer::{self, TIMER_FREQS, Timer},
    },
    peripherals::LEDC,
    system::{Peripheral, PeripheralGuard},
};

/// Duty fade errors
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FadeError {
    /// Duty change from start to end is out of range
    DutyRange,
    /// Duration too long for timer frequency and duty resolution
    Duration,
}

impl Display for FadeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::DutyRange => write!(f, "Duty change from start to end is out of range"),
            Self::Duration => write!(
                f,
                "Duration too long for timer frequency and duty resolution"
            ),
        }
    }
}

impl core::error::Error for FadeError {}

/// Channel configuration errors
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, _f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {}
    }
}

impl core::error::Error for ConfigError {}

/// Channel configuration
#[instability::unstable]
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The duty value
    duty: u32,
}

/// Channel number
#[instability::unstable]
#[derive(PartialEq, Eq, Copy, Clone, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Number {
    /// Channel 0
    Channel0 = 0,
    /// Channel 1
    Channel1 = 1,
    /// Channel 2
    Channel2 = 2,
    /// Channel 3
    Channel3 = 3,
    /// Channel 4
    Channel4 = 4,
    /// Channel 5
    Channel5 = 5,
    #[cfg(ledc_channel_count = "8")]
    /// Channel 6
    Channel6 = 6,
    #[cfg(ledc_channel_count = "8")]
    /// Channel 7
    Channel7 = 7,
}

impl Number {
    const fn from_u8(n: u8) -> Number {
        // until rust adds const enum generics
        match n {
            0 => Number::Channel0,
            1 => Number::Channel1,
            2 => Number::Channel2,
            3 => Number::Channel3,
            4 => Number::Channel4,
            5 => Number::Channel5,
            #[cfg(ledc_channel_count = "8")]
            6 => Number::Channel6,
            #[cfg(ledc_channel_count = "8")]
            7 => Number::Channel7,
            _ => core::unreachable!(), // defmt::unreachable!() fails const eval
        }
    }
}

/// Channel creator
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelCreator<'d, const CHANNEL: u8, Dm: DriverMode, S: Speed> {
    guard: Option<PeripheralGuard>,
    _phantom: PhantomData<(&'d (), Dm, S)>,
}

impl<'d, const CHANNEL: u8, Dm: DriverMode, S: Speed> ChannelCreator<'d, CHANNEL, Dm, S> {
    /// Reborrow this channel creator for a shorter lifetime `'a`.
    ///
    /// Use this method if you would like to keep working with this channel after you drop the
    /// configured one.
    #[instability::unstable]
    #[inline]
    pub fn reborrow(&mut self) -> ChannelCreator<'_, CHANNEL, Dm, S> {
        Self {
            guard: None,
            _phantom: PhantomData,
        }
    }

    /// Configures the channel.
    #[instability::unstable]
    pub fn configure(
        self,
        timer: &Timer<'d, S>,
        config: Config,
    ) -> Result<Channel<'d, Dm, S>, ConfigError> {
        let mut channel = Channel {
            number: Number::from_u8(CHANNEL),
            timer: timer.number(),
            pin: PinGuard::new_unconnected(),
            _guard: self.guard,
            _phantom: PhantomData,
        }
        .with_timer(timer);
        channel.apply_config(&config)?;

        Ok(channel)
    }

    /// Unsafely steal a channel creator instance.
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a channel is in use at one time.
    #[instability::unstable]
    #[inline]
    pub unsafe fn steal() -> Self {
        Self {
            guard: Some(PeripheralGuard::new(Peripheral::Ledc)),
            _phantom: PhantomData,
        }
    }

    /// Unsafely clone a channel creator instance.
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a channel is in use at one time.
    #[instability::unstable]
    #[inline]
    pub unsafe fn clone_unchecked(&self) -> Self {
        unsafe { Self::steal() }
    }
}

/// Channel struct
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channel<'d, Dm: DriverMode, S: Speed> {
    number: Number,
    timer: timer::Number,
    pin: PinGuard,
    _guard: Option<PeripheralGuard>,
    _phantom: PhantomData<(&'d (), Dm, S)>,
}

impl<'d, Dm: DriverMode, S: Speed> Channel<'d, Dm, S> {
    #[procmacros::doc_replace]
    /// Attaches a new timer to the channel and returns the updated channel.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::ledc::{self, Ledc};
    /// # use esp_hal::time::Rate;
    /// # let mut ledc = Ledc::new(peripherals.LEDC, Default::default())?;
    /// # let timer0 = ledc.timer0.configure(ledc::timer::Config::default().with_frequency(Rate::from_khz(24)))?;
    /// # let timer1 = ledc.timer1.configure(ledc::timer::Config::default().with_frequency(Rate::from_khz(12)))?;
    /// let channel0 = ledc.channel0.configure(&timer0, ledc::channel::Config::default())?;
    /// // timer0 is borrowed
    ///
    /// let channel0 = channel0.with_timer(&timer1);
    /// // timer0 is no longer borrowed, but timer1 is
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn with_timer(mut self, new_timer: &Timer<'d, S>) -> Self {
        let ledc = LEDC::regs();
        low_level::set_channel(ledc, self.number, new_timer.number() as u8, S::IS_HS);

        self.timer = new_timer.number();
        self
    }

    /// Attaches a new PeripheralOutput to the channel.
    #[instability::unstable]
    pub fn with_pin(mut self, pin: impl PeripheralOutput<'d>) -> Self {
        let output_signal = low_level::output_signal(self.number, S::IS_HS);
        let pin_out = pin.into();
        pin_out.apply_output_config(&Default::default());
        pin_out.set_output_enable(true);
        self.pin = pin_out.connect_with_guard(output_signal);
        self
    }

    /// Changes the configuration.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.set_duty_cycle(config.duty);
        Ok(())
    }

    /// Returns the maximum duty cycle.
    #[instability::unstable]
    pub fn max_duty_cycle(&self) -> u32 {
        let ledc = LEDC::regs();
        let duty_res = low_level::get_duty_res(ledc, self.timer, S::IS_HS);
        1u32 << duty_res
    }

    /// Sets the duty cycle of the channel.
    ///
    /// The duty cycle value will be clamped to the maximum duty cycle configured for the [`Timer`]
    #[instability::unstable]
    pub fn set_duty_cycle(&self, duty: u32) {
        let max_duty = self.max_duty_cycle();
        let duty_value = duty.min(max_duty);

        let ledc = LEDC::regs();
        low_level::set_duty_hw(ledc, self.number, S::IS_HS, duty_value);
        low_level::start_duty_without_fading(ledc, self.number, S::IS_HS);
        low_level::update_channel(ledc, self.number, S::IS_HS);
    }

    #[procmacros::doc_replace]
    /// Starts a duty fade from one duty value to another.
    ///
    /// There's a constraint on the combination of timer frequency, timer PWM
    /// duty resolution (the bit count), the fade "range" (abs(start-end)), and
    /// the duration:
    ///
    /// `frequency * duration / ((1<<bit_count) * abs(start-end)) < 1024`
    ///
    /// Small percentage changes, long durations, coarse PWM resolutions (that
    /// is, low bit counts), and high timer frequencies will all be more likely
    /// to fail this requirement. If it does fail, this function will return
    /// an error Result.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::ledc::Ledc;
    /// # let mut ledc = Ledc::new(peripherals.LEDC, Default::default())?;
    /// # let timer0 = ledc.timer0.configure(Default::default())?;
    /// # let mut channel0 = ledc.channel0.configure(&timer0, Default::default())?;
    /// let max_duty = channel0.max_duty_cycle();
    ///
    /// // Fade from 0 to 100% over 1000 milliseconds
    /// channel0.start_duty_fade(0, max_duty, 1000)?;
    ///
    /// // Wait for the fade to complete
    /// while channel0.is_duty_fade_running() {}
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn start_duty_fade(
        &self,
        start_duty: u32,
        end_duty: u32,
        duration_ms: u16,
    ) -> Result<(), FadeError> {
        let max_duty = self.max_duty_cycle();
        let start_duty_value = start_duty.min(max_duty);
        let end_duty_value = end_duty.min(max_duty);

        let timer_index = if S::IS_HS { 4 } else { 0 } + self.timer as usize;
        let frequency = TIMER_FREQS[timer_index].load(Ordering::Acquire);
        let pwm_cycles = duration_ms as u32 * frequency / 1000;
        let abs_duty_diff = end_duty_value.abs_diff(start_duty_value);
        let duty_steps: u32 = u16::try_from(abs_duty_diff).unwrap_or(65535).into();
        let duty_steps = duty_steps.min(pwm_cycles).max(1);

        let cycles_per_step =
            u16::try_from(pwm_cycles / duty_steps).map_err(|_| FadeError::Duration)?;
        if cycles_per_step > 1023 {
            return Err(FadeError::Duration);
        }

        let duty_per_cycle =
            u16::try_from(abs_duty_diff / duty_steps).map_err(|_| FadeError::DutyRange)?;

        let ledc = LEDC::regs();
        low_level::start_duty_fade_hw(
            ledc,
            self.number,
            S::IS_HS,
            start_duty_value,
            end_duty_value > start_duty_value,
            duty_steps as u16,
            cycles_per_step,
            duty_per_cycle,
        );
        low_level::update_channel(ledc, self.number, S::IS_HS);

        Ok(())
    }

    /// Returns true if a duty-cycle fade is running.
    #[instability::unstable]
    pub fn is_duty_fade_running(&self) -> bool {
        let ledc = LEDC::regs();
        low_level::is_duty_fade_running_hw(ledc, self.number, S::IS_HS)
    }
}

mod ehal1 {
    use embedded_hal::pwm::{ErrorType, SetDutyCycle};

    use super::Channel;
    use crate::{DriverMode, ledc::Speed};

    impl<Dm: DriverMode, S: Speed> ErrorType for Channel<'_, Dm, S> {
        type Error = core::convert::Infallible;
    }

    impl<Dm: DriverMode, S: Speed> SetDutyCycle for Channel<'_, Dm, S> {
        fn max_duty_cycle(&self) -> u16 {
            let max_hw = Self::max_duty_cycle(self); // inherent method
            if max_hw > u16::MAX as u32 {
                u16::MAX
            } else {
                max_hw as u16
            }
        }

        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            let max_duty = Self::max_duty_cycle(self); // inherent method
            let duty_to_set = if max_duty > u16::MAX as u32 {
                // Scale the u16 duty fraction to the hardware's internal u32 resolution
                ((duty as u64 * max_duty as u64) / u16::MAX as u64) as u32
            } else {
                duty as u32
            };
            Self::set_duty_cycle(self, duty_to_set); // inherent method
            Ok(())
        }
    }
}

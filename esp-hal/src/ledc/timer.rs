//! # LEDC timer
//!
//! ## Overview
//! The LEDC Timer provides a high-level interface to configure and control
//! individual timers of the `LEDC` peripheral.
//!
//! ## Configuration
//! The module allows precise and flexible control over timer configurations,
//! duty cycles and frequencies, making it ideal for Pulse-Width Modulation
//! (PWM) applications and LED lighting control.
//!
//! LEDC uses APB as clock source.
//!
//! For more information, please refer to the
#![doc = crate::trm_markdown_link!("ledpwm")]

use core::{fmt::Display, marker::PhantomData};

use crate::{
    ledc::{Speed, low_level},
    peripherals::LEDC,
    system::{Peripheral, PeripheralGuard},
    time::Rate,
};

const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;

/// Clock source for LEDC Timers
#[instability::unstable]
#[derive(Default, PartialEq, Eq, Copy, Clone, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// APB clock
    #[default]
    APBClock,
    // TODO: SLOWClk, REF_TICK
}

/// Timer number
#[instability::unstable]
#[derive(PartialEq, Eq, Copy, Clone, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Number {
    /// Timer 0
    Timer0 = 0,
    /// Timer 1
    Timer1 = 1,
    /// Timer 2
    Timer2 = 2,
    /// Timer 3
    Timer3 = 3,
}

impl Number {
    const fn from_u8(n: u8) -> Self {
        // until rust adds const enum generics
        match n {
            0 => Number::Timer0,
            1 => Number::Timer1,
            2 => Number::Timer2,
            3 => Number::Timer3,
            _ => core::unreachable!(), // defmt::unreachable!() fails const eval
        }
    }
}

/// Number of bits reserved for duty cycle adjustment
#[instability::unstable]
#[derive(PartialEq, Eq, Copy, Clone, Debug, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Duty {
    /// 1-bit resolution for duty cycle adjustment.
    Bit1 = 1,
    /// 2-bit resolution for duty cycle adjustment.
    Bit2,
    /// 3-bit resolution for duty cycle adjustment.
    Bit3,
    /// 4-bit resolution for duty cycle adjustment.
    Bit4,
    /// 5-bit resolution for duty cycle adjustment.
    Bit5,
    /// 6-bit resolution for duty cycle adjustment.
    Bit6,
    /// 7-bit resolution for duty cycle adjustment.
    Bit7,
    /// 8-bit resolution for duty cycle adjustment.
    Bit8,
    /// 9-bit resolution for duty cycle adjustment.
    Bit9,
    /// 10-bit resolution for duty cycle adjustment.
    Bit10,
    /// 11-bit resolution for duty cycle adjustment.
    Bit11,
    /// 12-bit resolution for duty cycle adjustment.
    Bit12,
    /// 13-bit resolution for duty cycle adjustment.
    Bit13,
    /// 14-bit resolution for duty cycle adjustment.
    Bit14,
    #[cfg(ledc_version = "1")]
    /// 15-bit resolution for duty cycle adjustment.
    Bit15,
    #[cfg(ledc_version = "1")]
    /// 16-bit resolution for duty cycle adjustment.
    Bit16,
    #[cfg(ledc_version = "1")]
    /// 17-bit resolution for duty cycle adjustment.
    Bit17,
    #[cfg(ledc_version = "1")]
    /// 18-bit resolution for duty cycle adjustment.
    Bit18,
    #[cfg(ledc_version = "1")]
    /// 19-bit resolution for duty cycle adjustment.
    Bit19,
    #[cfg(ledc_version = "1")]
    /// 20-bit resolution for duty cycle adjustment.
    Bit20,
}

impl TryFrom<u32> for Duty {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            1 => Self::Bit1,
            2 => Self::Bit2,
            3 => Self::Bit3,
            4 => Self::Bit4,
            5 => Self::Bit5,
            6 => Self::Bit6,
            7 => Self::Bit7,
            8 => Self::Bit8,
            9 => Self::Bit9,
            10 => Self::Bit10,
            11 => Self::Bit11,
            12 => Self::Bit12,
            13 => Self::Bit13,
            14 => Self::Bit14,
            #[cfg(ledc_version = "1")]
            15 => Self::Bit15,
            #[cfg(ledc_version = "1")]
            16 => Self::Bit16,
            #[cfg(ledc_version = "1")]
            17 => Self::Bit17,
            #[cfg(ledc_version = "1")]
            18 => Self::Bit18,
            #[cfg(ledc_version = "1")]
            19 => Self::Bit19,
            #[cfg(ledc_version = "1")]
            20 => Self::Bit20,
            _ => Err(())?,
        })
    }
}

/// Timer configuration errors
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Invalid Divisor
    Divisor,
}

impl Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Divisor => write!(f, "Invalid Divisor"),
        }
    }
}

impl core::error::Error for ConfigError {}

/// Timer configuration
#[instability::unstable]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The duty cycle resolution
    duty: Duty,
    /// The clock source for the timer
    clock_source: ClockSource,
    /// The frequency of the PWM signal in Hertz
    frequency: Rate,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            duty: Duty::Bit8,
            clock_source: ClockSource::default(),
            frequency: Rate::from_khz(1),
        }
    }
}

/// Timer creator
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimerCreator<'d, const TIMER: u8, S: Speed> {
    guard: Option<PeripheralGuard>,
    _phantom: PhantomData<(&'d (), S)>,
}

impl<'d, const TIMER: u8, S: Speed> TimerCreator<'d, TIMER, S> {
    /// Reborrow this timer creator for a shorter lifetime `'a`.
    ///
    /// Use this method if you would like to keep working with this timer after you drop the
    /// configured one.
    #[instability::unstable]
    #[inline]
    pub fn reborrow(&mut self) -> TimerCreator<'_, TIMER, S> {
        Self {
            guard: None,
            _phantom: PhantomData,
        }
    }

    /// Configures the timer.
    #[instability::unstable]
    pub fn configure(self, config: Config) -> Result<Timer<'d, S>, ConfigError> {
        let number = Number::from_u8(TIMER);

        let mut timer = Timer {
            number,
            config,
            #[cfg(soc_has_clock_node_ref_tick)]
            use_ref_tick: false,
            _guard: self.guard,
            _phantom: PhantomData,
        };

        timer.apply_config(&config)?;

        Ok(timer)
    }

    /// Unsafely steal a timer creator instance.
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a timer is in use at one time.
    #[instability::unstable]
    #[inline]
    pub unsafe fn steal() -> Self {
        Self {
            guard: Some(PeripheralGuard::new(Peripheral::Ledc)),
            _phantom: PhantomData,
        }
    }

    /// Unsafely clone a timer creator instance.
    ///
    /// # Safety
    ///
    /// The caller must ensure that only one instance of a timer is in use at one time.
    #[instability::unstable]
    #[inline]
    pub unsafe fn clone_unchecked(&self) -> Self {
        unsafe { Self::steal() }
    }
}

fn apply_config_ls(number: Number, config: &Config) -> Result<bool, ConfigError> {
    let src_freq: u32 = low_level::ls_freq_hw(config.clock_source).as_hz();
    let precision = 1 << config.duty as u32;
    let frequency: u32 = config.frequency.as_hz();

    #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused_mut))]
    let mut divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;

    #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused_mut))]
    let mut use_ref_tick = false;

    #[cfg(soc_has_clock_node_ref_tick)]
    if divisor > LEDC_TIMER_DIV_NUM_MAX {
        // APB_CLK results in divisor which is too high. Try using REF_TICK as clock
        // source.
        use_ref_tick = true;
        divisor = (1_000_000u64 << 8) / frequency as u64 / precision as u64;
    }

    if !(256..=LEDC_TIMER_DIV_NUM_MAX).contains(&divisor) {
        return Err(ConfigError::Divisor);
    }

    let ledc = LEDC::regs();
    low_level::ls_configure_hw(
        ledc,
        number,
        divisor as u32,
        config.duty as u8,
        use_ref_tick,
    );
    low_level::ls_update_hw(ledc, number);

    Ok(use_ref_tick)
}

#[cfg(ledc_version = "1")]
fn apply_config_hs(number: Number, config: &Config) -> Result<bool, ConfigError> {
    let src_freq: u32 = low_level::hs_freq_hw(config.clock_source).as_hz();
    let precision = 1 << config.duty as u32;
    let frequency: u32 = config.frequency.as_hz();

    let divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;

    if !(256..=LEDC_TIMER_DIV_NUM_MAX).contains(&divisor) {
        return Err(ConfigError::Divisor);
    }

    let ledc = LEDC::regs();
    low_level::hs_configure_hw(
        ledc,
        number,
        divisor as u32,
        config.duty as u8,
        config.clock_source,
    );
    low_level::hs_update_hw();

    Ok(false)
}

/// Timer struct
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<'d, S: Speed> {
    number: Number,
    config: Config,
    #[cfg(soc_has_clock_node_ref_tick)]
    use_ref_tick: bool,
    _guard: Option<PeripheralGuard>,
    _phantom: PhantomData<(&'d (), S)>,
}

impl<'d, S: Speed> Timer<'d, S> {
    /// Changes the configuration.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        #[cfg_attr(not(soc_has_clock_node_ref_tick), expect(unused))]
        let use_ref_tick = if S::IS_HS {
            cfg_select! {
                ledc_version = "1" => apply_config_hs(self.number, config),
                _ => unreachable!()
            }
        } else {
            apply_config_ls(self.number, config)
        }?;

        self.config = *config;
        #[cfg(soc_has_clock_node_ref_tick)]
        {
            self.use_ref_tick = use_ref_tick;
        }

        Ok(())
    }

    /// Returns the configuration of the timer.
    #[instability::unstable]
    #[inline]
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Returns the number of the timer.
    #[instability::unstable]
    #[inline]
    pub fn number(&self) -> Number {
        self.number
    }
}

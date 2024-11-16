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

use fugit::HertzU32;

#[cfg(esp32)]
use super::HighSpeed;
use super::{LowSpeed, Speed};
use crate::{clock::Clocks, peripherals::ledc};

const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;

/// Timer errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid Divisor
    Divisor,
}

#[cfg(esp32)]
/// Clock source for HS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HSClockSource {
    /// APB clock.
    APBClk,
    // TODO RefTick,
}

/// Clock source for LS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LSClockSource {
    /// APB clock.
    APBClk,
    // TODO SLOWClk
}

/// Timer number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Number {
    /// Timer 0.
    Timer0 = 0,
    /// Timer 1.
    Timer1 = 1,
    /// Timer 2.
    Timer2 = 2,
    /// Timer 3.
    Timer3 = 3,
}

/// Timer configuration
pub mod config {
    use fugit::HertzU32;

    /// Number of bits reserved for duty cycle adjustment
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Duty {
        /// 1-bit resolution for duty cycle adjustment.
        Duty1Bit = 1,
        /// 2-bit resolution for duty cycle adjustment.
        Duty2Bit,
        /// 3-bit resolution for duty cycle adjustment.
        Duty3Bit,
        /// 4-bit resolution for duty cycle adjustment.
        Duty4Bit,
        /// 5-bit resolution for duty cycle adjustment.
        Duty5Bit,
        /// 6-bit resolution for duty cycle adjustment.
        Duty6Bit,
        /// 7-bit resolution for duty cycle adjustment.
        Duty7Bit,
        /// 8-bit resolution for duty cycle adjustment.
        Duty8Bit,
        /// 9-bit resolution for duty cycle adjustment.
        Duty9Bit,
        /// 10-bit resolution for duty cycle adjustment.
        Duty10Bit,
        /// 11-bit resolution for duty cycle adjustment.
        Duty11Bit,
        /// 12-bit resolution for duty cycle adjustment.
        Duty12Bit,
        /// 13-bit resolution for duty cycle adjustment.
        Duty13Bit,
        /// 14-bit resolution for duty cycle adjustment.
        Duty14Bit,
        #[cfg(esp32)]
        /// 15-bit resolution for duty cycle adjustment.
        Duty15Bit,
        #[cfg(esp32)]
        /// 16-bit resolution for duty cycle adjustment.
        Duty16Bit,
        #[cfg(esp32)]
        /// 17-bit resolution for duty cycle adjustment.
        Duty17Bit,
        #[cfg(esp32)]
        /// 18-bit resolution for duty cycle adjustment.
        Duty18Bit,
        #[cfg(esp32)]
        /// 19-bit resolution for duty cycle adjustment.
        Duty19Bit,
        #[cfg(esp32)]
        /// 20-bit resolution for duty cycle adjustment.
        Duty20Bit,
    }

    impl TryFrom<u32> for Duty {
        type Error = ();

        fn try_from(value: u32) -> Result<Self, Self::Error> {
            Ok(match value {
                1 => Self::Duty1Bit,
                2 => Self::Duty2Bit,
                3 => Self::Duty3Bit,
                4 => Self::Duty4Bit,
                5 => Self::Duty5Bit,
                6 => Self::Duty6Bit,
                7 => Self::Duty7Bit,
                8 => Self::Duty8Bit,
                9 => Self::Duty9Bit,
                10 => Self::Duty10Bit,
                11 => Self::Duty11Bit,
                12 => Self::Duty12Bit,
                13 => Self::Duty13Bit,
                14 => Self::Duty14Bit,
                #[cfg(esp32)]
                15 => Self::Duty15Bit,
                #[cfg(esp32)]
                16 => Self::Duty16Bit,
                #[cfg(esp32)]
                17 => Self::Duty17Bit,
                #[cfg(esp32)]
                18 => Self::Duty18Bit,
                #[cfg(esp32)]
                19 => Self::Duty19Bit,
                #[cfg(esp32)]
                20 => Self::Duty20Bit,
                _ => Err(())?,
            })
        }
    }

    /// Timer configuration
    #[derive(Copy, Clone)]
    pub struct Config<CS> {
        /// The duty cycle resolution.
        pub duty: Duty,
        /// The clock source for the timer.
        pub clock_source: CS,
        /// The frequency of the PWM signal in Hertz.
        pub frequency: HertzU32,
    }
}

/// Trait defining the type of timer source
pub trait TimerSpeed: Speed {
    /// The type of clock source used by the timer in this speed mode.
    type ClockSourceType;
}

/// Timer source type for LowSpeed timers
impl TimerSpeed for LowSpeed {
    /// The clock source type for low-speed timers.
    type ClockSourceType = LSClockSource;
}

#[cfg(esp32)]
/// Timer source type for HighSpeed timers
impl TimerSpeed for HighSpeed {
    /// The clock source type for high-speed timers.
    type ClockSourceType = HSClockSource;
}

/// Interface for Timers
pub trait TimerIFace<S: TimerSpeed> {
    /// Return the frequency of the timer
    fn freq(&self) -> Option<HertzU32>;

    /// Configure the timer
    fn configure(&mut self, config: config::Config<S::ClockSourceType>) -> Result<(), Error>;

    /// Check if the timer has been configured
    fn is_configured(&self) -> bool;

    /// Return the duty resolution of the timer
    fn duty(&self) -> Option<config::Duty>;

    /// Return the timer number
    fn number(&self) -> Number;

    /// Return the timer frequency, or 0 if not configured
    fn frequency(&self) -> u32;
}

/// Interface for HW configuration of timer
pub trait TimerHW<S: TimerSpeed> {
    /// Get the current source timer frequency from the HW
    fn freq_hw(&self) -> Option<HertzU32>;

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32);

    /// Update the timer in HW
    fn update_hw(&self);
}

/// Timer struct
pub struct Timer<'a, S: TimerSpeed> {
    ledc: &'a crate::peripherals::ledc::RegisterBlock,
    number: Number,
    duty: Option<config::Duty>,
    frequency: u32,
    configured: bool,
    use_ref_tick: bool,
    clock_source: Option<S::ClockSourceType>,
}

impl<'a, S: TimerSpeed> TimerIFace<S> for Timer<'a, S>
where
    Timer<'a, S>: TimerHW<S>,
{
    /// Return the frequency of the timer
    fn freq(&self) -> Option<HertzU32> {
        self.freq_hw()
    }

    /// Configure the timer
    fn configure(&mut self, config: config::Config<S::ClockSourceType>) -> Result<(), Error> {
        self.duty = Some(config.duty);
        self.clock_source = Some(config.clock_source);

        // TODO: we should return some error here if `unwrap()` fails
        let src_freq: u32 = self.freq().unwrap().to_Hz();
        let precision = 1 << config.duty as u32;
        let frequency: u32 = config.frequency.raw();
        self.frequency = frequency;

        let mut divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;

        if divisor > LEDC_TIMER_DIV_NUM_MAX {
            // APB_CLK results in divisor which too high. Try using REF_TICK as clock
            // source.
            self.use_ref_tick = true;
            divisor = (1_000_000u64 << 8) / frequency as u64 / precision as u64;
        }

        if !(256..LEDC_TIMER_DIV_NUM_MAX).contains(&divisor) {
            return Err(Error::Divisor);
        }

        self.configure_hw(divisor as u32);
        self.update_hw();

        self.configured = true;

        Ok(())
    }

    /// Check if the timer has been configured
    fn is_configured(&self) -> bool {
        self.configured
    }

    /// Return the duty resolution of the timer
    fn duty(&self) -> Option<config::Duty> {
        self.duty
    }

    /// Return the timer number
    fn number(&self) -> Number {
        self.number
    }

    /// Return the timer frequency
    fn frequency(&self) -> u32 {
        self.frequency
    }
}

impl<'a, S: TimerSpeed> Timer<'a, S> {
    /// Create a new instance of a timer
    pub fn new(ledc: &'a ledc::RegisterBlock, number: Number) -> Self {
        Timer {
            ledc,
            number,
            duty: None,
            frequency: 0u32,
            configured: false,
            use_ref_tick: false,
            clock_source: None,
        }
    }
}

/// Timer HW implementation for LowSpeed timers
impl TimerHW<LowSpeed> for Timer<'_, LowSpeed> {
    /// Get the current source timer frequency from the HW
    fn freq_hw(&self) -> Option<HertzU32> {
        self.clock_source.map(|source| match source {
            LSClockSource::APBClk => {
                let clocks = Clocks::get();
                clocks.apb_clock
            }
        })
    }

    #[cfg(esp32)]
    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = unwrap!(self.duty) as u8;
        let use_apb = !self.use_ref_tick;

        self.ledc
            .lstimer(self.number as usize)
            .conf()
            .modify(|_, w| unsafe {
                w.tick_sel().bit(use_apb);
                w.rst().clear_bit();
                w.pause().clear_bit();
                w.div_num().bits(divisor);
                w.duty_res().bits(duty)
            });
    }

    #[cfg(not(esp32))]
    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = unwrap!(self.duty) as u8;
        let use_ref_tick = self.use_ref_tick;

        self.ledc
            .timer(self.number as usize)
            .conf()
            .modify(|_, w| unsafe {
                w.tick_sel().bit(use_ref_tick);
                w.rst().clear_bit();
                w.pause().clear_bit();
                w.clk_div().bits(divisor);
                w.duty_res().bits(duty)
            });
    }

    /// Update the timer in HW
    fn update_hw(&self) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let tmr = self.ledc.lstimer(self.number as usize);
            } else {
                let tmr = self.ledc.timer(self.number as usize);
            }
        }

        tmr.conf().modify(|_, w| w.para_up().set_bit());
    }
}

#[cfg(esp32)]
/// Timer HW implementation for HighSpeed timers
impl<'a> TimerHW<HighSpeed> for Timer<'a, HighSpeed> {
    /// Get the current source timer frequency from the HW
    fn freq_hw(&self) -> Option<HertzU32> {
        self.clock_source.map(|source| match source {
            HSClockSource::APBClk => {
                let clocks = Clocks::get();
                clocks.apb_clock
            }
        })
    }

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = unwrap!(self.duty) as u8;
        let sel_hstimer = self.clock_source == Some(HSClockSource::APBClk);

        self.ledc
            .hstimer(self.number as usize)
            .conf()
            .modify(|_, w| unsafe {
                w.tick_sel().bit(sel_hstimer);
                w.rst().clear_bit();
                w.pause().clear_bit();
                w.div_num().bits(divisor);
                w.duty_res().bits(duty)
            });
    }

    /// Update the timer in HW
    fn update_hw(&self) {
        // Nothing to do for HS timers
    }
}

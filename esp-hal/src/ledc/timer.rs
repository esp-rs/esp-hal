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

use core::marker::PhantomData;

use config::Duty;
use fugit::HertzU32;

#[cfg(esp32)]
use super::HighSpeed;
use super::{Ledc, LowSpeed, Speed};
use crate::clock::Clocks;

const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;

/// Timer errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid Divisor
    Divisor,
}

/// This trait describes the clock source for one specific timer
/// In esp32 and esp32s2, there is a mux to select the clock source in each
/// timer, which is decided by `LEDC_TICK_SEL_TIMERx` register.
/// If the value is 0, the clock source is the LEDC_PWM_CLK, which is configured
/// by the `LEDC_APB_CLK_SEL` register. Otherwise, the clock source is the
/// REF_TICK.
pub trait ClockSource: Sized {
    #[cfg(any(esp32, esp32s2))]
    /// REF_TICK clock is used as the backup clock source for timers when the
    /// divisor is too high.
    const REF_TICK: Self;
    /// Get the clock frequency of the clock source
    fn get_clock_freq(&self) -> HertzU32;
}

#[cfg(esp32)]
/// Clock source for HS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HSClockSource {
    /// LEDC_PWM_CLK
    LedcPwmClk,
    /// REF_TICK.
    RefTick,
}

#[cfg(esp32)]
impl ClockSource for HSClockSource {
    /// Get the clock frequency of the clock source
    /// esp32 has two clock sources for HS timers: APB_CLK and REF_TICK
    fn get_clock_freq(&self) -> HertzU32 {
        match self {
            HSClockSource::LedcPwmClk => {
                let clocks = Clocks::get();
                clocks.apb_clock
            }
            HSClockSource::RefTick => HertzU32::from(crate::soc::constants::REF_TICK),
        }
    }
    const REF_TICK: Self = Self::RefTick;
}

/// Clock source for LS Timers
/// Only esp32 and esp32s2 have REF_TICK
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LSClockSource {
    /// LEDC_PWM_CLK
    LedcPwmClk,
    #[cfg(any(esp32, esp32s2))]
    /// REF_TICK clock.
    RefTick,
}

impl ClockSource for LSClockSource {
    /// Get the clock frequency of the clock source
    /// For slow-speed timers, there are multiple clock sources:
    /// For example, if LEDC_APB_CLK_SEL is set to 1, the clock source is
    /// APB_CLK. if LEDC_APB_CLK_SEL is set to 2, the clock source is
    /// RC_FAST_CLK. if LEDC_APB_CLK_SEL is set to 3, the clock source is
    /// XTAL_CLK. Currently, only APB_CLK is supported.
    fn get_clock_freq(&self) -> HertzU32 {
        match self {
            LSClockSource::LedcPwmClk => {
                let source = Ledc::get_global_slow_clock().expect("Global slow clock not set.");
                match source {
                    super::LSGlobalClkSource::APBClk => {
                        let clocks = Clocks::get();
                        clocks.apb_clock
                    }
                }
            }
            #[cfg(any(esp32, esp32s2))]
            LSClockSource::RefTick => HertzU32::from(crate::soc::constants::REF_TICK),
        }
    }
    #[cfg(any(esp32, esp32s2))]
    const REF_TICK: Self = Self::RefTick;
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
        #[cfg(any(esp32, esp32c6, esp32h2))]
        /// 15-bit resolution for duty cycle adjustment.
        Duty15Bit,
        #[cfg(any(esp32, esp32c6, esp32h2))]
        /// 16-bit resolution for duty cycle adjustment.
        Duty16Bit,
        #[cfg(any(esp32, esp32c6, esp32h2))]
        /// 17-bit resolution for duty cycle adjustment.
        Duty17Bit,
        #[cfg(any(esp32, esp32c6, esp32h2))]
        /// 18-bit resolution for duty cycle adjustment.
        Duty18Bit,
        #[cfg(any(esp32, esp32c6, esp32h2))]
        /// 19-bit resolution for duty cycle adjustment.
        Duty19Bit,
        #[cfg(any(esp32, esp32c6, esp32h2))]
        /// 20-bit resolution for duty cycle adjustment.
        Duty20Bit,
    }

    impl TryFrom<u8> for Duty {
        type Error = ();

        fn try_from(value: u8) -> Result<Self, Self::Error> {
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
                #[cfg(any(esp32, esp32c6, esp32h2))]
                15 => Self::Duty15Bit,
                #[cfg(any(esp32, esp32c6, esp32h2))]
                16 => Self::Duty16Bit,
                #[cfg(any(esp32, esp32c6, esp32h2))]
                17 => Self::Duty17Bit,
                #[cfg(any(esp32, esp32c6, esp32h2))]
                18 => Self::Duty18Bit,
                #[cfg(any(esp32, esp32c6, esp32h2))]
                19 => Self::Duty19Bit,
                #[cfg(any(esp32, esp32c6, esp32h2))]
                20 => Self::Duty20Bit,
                _ => Err(())?,
            })
        }
    }
}

/// Trait defining the type of timer source
pub trait TimerSpeed: Speed {
    /// The type of clock source used by the timer in this speed mode.
    type ClockSourceType: ClockSource;

    #[cfg(any(esp32, esp32s2))]
    /// esp32 and esp32s2 have a mux to select the clock source in each timer,
    /// which allows the user to select the clock source for each timer.
    fn select_timer(number: Number, source: Self::ClockSourceType);
    /// Get the clock source of the timer
    fn get_clock_source(number: Number) -> Self::ClockSourceType;
    /// Get the raw duty number of the timer, the number can be 0 if the timer
    /// is not configured.
    fn get_duty_bits(number: Number) -> u8;
    /// Set the duty number of the timer
    fn set_duty(number: Number, duty: config::Duty);
    /// Get the raw divisor number of the timer, the number can be 0 if the
    /// timer is not configured.
    fn get_divisor_bits(number: Number) -> u32;
    /// Set the divisor number of the timer
    fn set_divisor(number: Number, divisor: u32);
    /// Set the pause bit of the timer
    fn set_pause_bit(number: Number, pause: bool);
    /// Set the reset bit of the timer
    fn set_reset_bit(number: Number, reset: bool);
    /// Update the parameters of the timer
    fn param_update(number: Number);
    /// Get the counter value of the timer
    fn get_counter(number: Number) -> u32;
}

/// Timer source type for LowSpeed timers
impl TimerSpeed for LowSpeed {
    /// The clock source type for low-speed timers.
    type ClockSourceType = LSClockSource;

    #[cfg(any(esp32, esp32s2))]
    fn get_clock_source(number: Number) -> Self::ClockSourceType {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        match timer.conf().read().tick_sel().bit() {
            true => LSClockSource::LedcPwmClk,
            false => LSClockSource::RefTick,
        }
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn get_clock_source(_number: Number) -> Self::ClockSourceType {
        LSClockSource::LedcPwmClk
    }

    #[cfg(any(esp32, esp32s2))]
    fn select_timer(number: Number, source: Self::ClockSourceType) {
        let source = match source {
            LSClockSource::LedcPwmClk => true,
            LSClockSource::RefTick => false,
        };
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer.conf().modify(|_, w| w.tick_sel().bit(source));
    }

    fn get_duty_bits(number: Number) -> u8 {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer.conf().read().duty_res().bits()
    }

    fn set_duty(number: Number, duty: config::Duty) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer
            .conf()
            .modify(|_, w| unsafe { w.duty_res().bits(duty as u8) });
    }

    #[cfg(not(esp32))]
    fn get_divisor_bits(number: Number) -> u32 {
        Ledc::register_block()
            .timer(number as usize)
            .conf()
            .read()
            .clk_div()
            .bits()
    }

    #[cfg(esp32)]
    fn get_divisor_bits(number: Number) -> u32 {
        Ledc::register_block()
            .lstimer(number as usize)
            .conf()
            .read()
            .div_num()
            .bits()
    }

    #[cfg(not(esp32))]
    fn set_divisor(number: Number, divisor: u32) {
        Ledc::register_block()
            .timer(number as usize)
            .conf()
            .modify(|_, w| unsafe { w.clk_div().bits(divisor) });
    }

    #[cfg(esp32)]
    fn set_divisor(number: Number, divisor: u32) {
        Ledc::register_block()
            .lstimer(number as usize)
            .conf()
            .modify(|_, w| unsafe { w.div_num().bits(divisor) });
    }

    fn set_pause_bit(number: Number, pause: bool) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer.conf().modify(|_, w| w.pause().bit(pause));
    }

    fn set_reset_bit(number: Number, reset: bool) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer.conf().modify(|_, w| w.rst().bit(reset));
    }

    fn param_update(number: Number) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer.conf().modify(|_, w| w.para_up().set_bit());
    }

    fn get_counter(number: Number) -> u32 {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let timer = Ledc::register_block()
                .lstimer(number as usize);
            } else {
                let timer = Ledc::register_block()
                .timer(number as usize);
            }
        };
        timer.value().read().cnt().bits() as _
    }
}

#[cfg(esp32)]
/// Timer source type for HighSpeed timers
impl TimerSpeed for HighSpeed {
    /// The clock source type for high-speed timers.
    type ClockSourceType = HSClockSource;

    fn get_clock_source(number: Number) -> Self::ClockSourceType {
        match Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .read()
            .tick_sel()
            .bit()
        {
            true => HSClockSource::LedcPwmClk,
            false => HSClockSource::RefTick,
        }
    }

    fn select_timer(number: Number, source: Self::ClockSourceType) {
        let source = match source {
            HSClockSource::LedcPwmClk => true,
            HSClockSource::RefTick => false,
        };
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .modify(|_, w| w.tick_sel().bit(source));
    }

    fn get_duty_bits(number: Number) -> u8 {
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .read()
            .duty_res()
            .bits()
    }

    fn set_duty(number: Number, duty: config::Duty) {
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .modify(|_, w| unsafe { w.duty_res().bits(duty as u8) });
    }

    fn get_divisor_bits(number: Number) -> u32 {
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .read()
            .div_num()
            .bits()
    }

    fn set_divisor(number: Number, divisor: u32) {
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .modify(|_, w| unsafe { w.div_num().bits(divisor) });
    }

    fn set_pause_bit(number: Number, pause: bool) {
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .modify(|_, w| w.pause().bit(pause));
    }

    fn set_reset_bit(number: Number, reset: bool) {
        Ledc::register_block()
            .hstimer(number as usize)
            .conf()
            .modify(|_, w| w.rst().bit(reset));
    }

    fn param_update(_: Number) {
        // The parameters will be updated in the next cycle
    }

    fn get_counter(number: Number) -> u32 {
        Ledc::register_block()
            .hstimer(number as usize)
            .value()
            .read()
            .cnt()
            .bits() as _
    }
}

/// Builder for Timer
pub struct TimerBuilder<S: TimerSpeed> {
    phantom: PhantomData<S>,
    number: Number,
}

impl<S: TimerSpeed> TimerBuilder<S> {
    /// Create a new instance of a timer builder
    /// `number`, `duty`, `clock_source` and `frequency` are the required
    /// parameters. other parameters can be set by calling the corresponding
    /// methods.
    pub fn new(
        _ledc: &Ledc<'_>,
        number: Number,
        duty: Duty,
        clock_source: S::ClockSourceType,
        frequency: HertzU32,
    ) -> Result<TimerBuilder<S>, Error> {
        S::set_duty(number, duty);
        let builder = TimerBuilder {
            phantom: PhantomData,
            number,
        };
        builder.set_frequency(duty, clock_source, frequency)
    }

    /// Create a new instance of a timer builder from a configured timer
    pub fn from_configured_timer(timer: Timer<S>) -> TimerBuilder<S> {
        TimerBuilder {
            phantom: PhantomData,
            number: timer.number,
        }
    }

    /// Set the frequency of the timer
    pub fn set_frequency(
        &self,
        duty: Duty,
        clock_source: S::ClockSourceType,
        frequency: HertzU32,
    ) -> Result<Self, Error> {
        let clock_frequency = clock_source.get_clock_freq();
        let precision = 1 << duty as u32;
        let frequency = frequency.to_Hz() as u64;

        #[cfg(any(esp32, esp32s2))]
        let mut clock_source = clock_source;

        #[cfg(any(esp32, esp32s2))]
        let mut divisor = ((clock_frequency.to_Hz() as u64) << 8) / frequency / precision as u64;

        #[cfg(not(any(esp32, esp32s2)))]
        let divisor = ((clock_frequency.to_Hz() as u64) << 8) / frequency / precision as u64;

        #[cfg(any(esp32, esp32s2))]
        if divisor > LEDC_TIMER_DIV_NUM_MAX {
            // APB_CLK results in divisor which too high. Try using REF_TICK as clock
            // source.
            clock_source = S::ClockSourceType::REF_TICK;
            divisor = (1_000_000u64 << 8) / frequency / precision as u64;
        }

        if !(256..LEDC_TIMER_DIV_NUM_MAX).contains(&divisor) {
            return Err(Error::Divisor);
        }

        #[cfg(any(esp32, esp32s2))]
        S::select_timer(self.number, clock_source);
        S::set_divisor(self.number, divisor as u32);
        Ok(TimerBuilder {
            phantom: PhantomData,
            number: self.number,
        })
    }

    /// pause the timer
    pub fn pause(self) -> Self {
        S::set_pause_bit(self.number, true);
        self
    }

    /// resume the timer
    pub fn resume(self) -> Self {
        S::set_pause_bit(self.number, false);
        self
    }

    /// reset the timer
    pub fn reset(self) -> Self {
        S::set_reset_bit(self.number, true);
        self
    }

    /// build the timer
    pub fn build(self) -> Timer<S> {
        S::param_update(self.number);
        S::set_reset_bit(self.number, false);
        Timer::new(self.number)
    }
}

/// Interface for Timers
pub trait TimerIFace<S: TimerSpeed>: Sync {
    /// Return the duty resolution of the timer
    fn get_duty(&self) -> Option<config::Duty>;

    /// Return the timer number
    fn get_number(&self) -> Number;

    /// Return the timer frequency, or 0 if not configured
    fn get_frequency(&self) -> u32;
}

/// Timer struct
#[derive(Clone, Copy)]
pub struct Timer<S: TimerSpeed> {
    phantom: PhantomData<S>,
    number: Number,
}

impl<S: TimerSpeed> TimerIFace<S> for Timer<S> {
    /// Return the duty resolution of the timer
    fn get_duty(&self) -> Option<config::Duty> {
        let duty = LowSpeed::get_duty_bits(self.number);
        if duty == 0 {
            return None;
        }
        Some(Duty::try_from(duty).expect("Unexpect duty value from the ledc register."))
    }

    /// Return the timer number
    fn get_number(&self) -> Number {
        self.number
    }

    /// Return the timer frequency
    fn get_frequency(&self) -> u32 {
        let clock = S::get_clock_source(self.number).get_clock_freq();
        let divisor = S::get_divisor_bits(self.number);
        if divisor == 0 {
            return 0;
        }
        let duty = match self.get_duty() {
            Some(duty) => duty,
            None => return 0,
        } as u8;
        let precision = 1 << duty as u64;
        (((clock.to_Hz() as u64) << 8) / precision / (divisor as u64)) as u32
    }
}

impl<S: TimerSpeed> Timer<S> {
    /// Create a new instance of a timer
    fn new(number: Number) -> Self {
        Timer {
            phantom: PhantomData,
            number,
        }
    }
}

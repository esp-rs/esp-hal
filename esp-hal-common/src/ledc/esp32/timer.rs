use super::{LowSpeed, Speed};
use crate::{clock::Clocks, pac::ledc, prelude::_fugit_MegahertzU32};
/// Timer errors
#[derive(Debug)]
pub enum Error {
    /// Invalid Divisor
    Divisor,
}

use super::HighSpeed;

/// Clock source for HS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum HSClockSource {
    // TODO RefTick,
    APBClk,
}

/// Clock source for LS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSClockSource {
    APBClk,
    // TODO SLOWClk
    // SLOWClk,
}

/// Timer number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Timer0,
    Timer1,
    Timer2,
    Timer3,
}

/// Timer configuration
pub mod config {

    use crate::prelude::_fugit_HertzU32;

    /// Number of bits reserved for duty cycle adjustment
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum Duty {
        Duty1Bit = 1,
        Duty2Bit,
        Duty3Bit,
        Duty4Bit,
        Duty5Bit,
        Duty6Bit,
        Duty7Bit,
        Duty8Bit,
        Duty9Bit,
        Duty10Bit,
        Duty11Bit,
        Duty12Bit,
        Duty13Bit,
        Duty14Bit,
        Duty15Bit,
        Duty16Bit,
        Duty17Bit,
        Duty18Bit,
        Duty19Bit,
        Duty20Bit,
    }

    /// Timer configuration
    #[derive(Copy, Clone)]
    pub struct Config<CS> {
        pub duty: Duty,
        pub clock_source: CS,
        pub frequency: _fugit_HertzU32,
    }
}

/// Trait defining the type of timer source
pub trait TimerSpeed: Speed {
    type ClockSourceType;
}

/// Timer source type for LowSpeed timers
impl TimerSpeed for LowSpeed {
    type ClockSourceType = LSClockSource;
}

/// Timer source type for HighSpeed timers
impl TimerSpeed for HighSpeed {
    type ClockSourceType = HSClockSource;
}

/// Interface for Timers
pub trait TimerIFace<S: TimerSpeed> {
    /// Return the frequency of the timer
    fn get_freq(&self) -> Option<_fugit_MegahertzU32>;

    /// Configure the timer
    fn configure(&mut self, config: config::Config<S::ClockSourceType>) -> Result<(), Error>;

    /// Check if the timer has been configured
    fn is_configured(&self) -> bool;

    /// Return the duty resolution of the timer
    fn get_duty(&self) -> Option<config::Duty>;

    /// Return the timer number
    fn get_number(&self) -> Number;
}

/// Interface for HW configuration of timer
pub trait TimerHW<S: TimerSpeed> {
    /// Get the current source timer frequency from the HW
    fn get_freq_hw(&self) -> Option<_fugit_MegahertzU32>;

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32);

    /// Update the timer in HW
    fn update_hw(&self);
}

/// Timer struct
pub struct Timer<'a, S: TimerSpeed> {
    ledc: &'a crate::pac::ledc::RegisterBlock,
    clock_control_config: &'a Clocks,
    number: Number,
    duty: Option<config::Duty>,
    configured: bool,
    clock_source: Option<S::ClockSourceType>,
}

impl<'a, S: TimerSpeed> TimerIFace<S> for Timer<'a, S>
where
    Timer<'a, S>: TimerHW<S>,
{
    /// Return the frequency of the timer
    fn get_freq(&self) -> Option<_fugit_MegahertzU32> {
        self.get_freq_hw()
    }

    /// Configure the timer
    fn configure(&mut self, config: config::Config<S::ClockSourceType>) -> Result<(), Error> {
        self.duty = Some(config.duty);
        self.clock_source = Some(config.clock_source);

        let src_freq: u32 = self.get_freq().unwrap().to_Hz();
        let precision = 2_u64.pow(config.duty as u32);
        let frequency: u32 = config.frequency.raw();

        let divisor = (((src_freq as u64) << 8) + ((frequency as u64 * precision) / 2))
            / (frequency as u64 * precision);

        if divisor >= 0x10_0000 || divisor == 0 {
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
    fn get_duty(&self) -> Option<config::Duty> {
        self.duty
    }

    /// Return the timer number
    fn get_number(&self) -> Number {
        self.number
    }
}

impl<'a, S: TimerSpeed> Timer<'a, S> {
    /// Create a new intance of a timer
    pub fn new(
        ledc: &'a ledc::RegisterBlock,
        clock_control_config: &'a Clocks,
        number: Number,
    ) -> Self {
        Timer {
            ledc,
            clock_control_config,
            number,
            duty: None,
            configured: false,
            clock_source: None,
        }
    }

    // TODO This function will be relevant when we add other clk sources
    // Helper function that return the current frequency of the LowSpeed global
    // source
    // fn get_slow_clock_freq(&self) -> _fugit_MegahertzU32 {
    //             if self.ledc.conf.read().apb_clk_sel().bit_is_clear() {
    //                 8u32.MHz().into()
    //             } else {
    //                 // 80u32.MHz().into()
    //                 self.clock_control_config.apb_clock
    //             }
    // }
}

/// Timer HW implementation for LowSpeed timers
impl<'a> TimerHW<LowSpeed> for Timer<'a, LowSpeed> {
    /// Get the current source timer frequency from the HW
    fn get_freq_hw(&self) -> Option<_fugit_MegahertzU32> {
        self.clock_source.map(|cs| match cs {
            LSClockSource::APBClk => self.clock_control_config.apb_clock,
        })
    }

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = self.duty.unwrap() as u8;
        let sel_lstimer = self.clock_source.unwrap() == LSClockSource::APBClk;
        match self.number {
            Number::Timer0 => self.ledc.lstimer0_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_lstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer1 => self.ledc.lstimer1_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_lstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer2 => self.ledc.lstimer2_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_lstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer3 => self.ledc.lstimer3_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_lstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
        };
    }

    /// Update the timer in HW
    fn update_hw(&self) {
        match self.number {
            Number::Timer0 => self.ledc.lstimer0_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer1 => self.ledc.lstimer1_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer2 => self.ledc.lstimer2_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer3 => self.ledc.lstimer3_conf.modify(|_, w| w.para_up().set_bit()),
        };
    }
}

/// Timer HW implementation for HighSpeed timers
impl<'a> TimerHW<HighSpeed> for Timer<'a, HighSpeed> {
    /// Get the current source timer frequency from the HW
    fn get_freq_hw(&self) -> Option<_fugit_MegahertzU32> {
        self.clock_source.map(|cs| match cs {
            // TODO RefTick HSClockSource::RefTick => self.clock_control_config.apb_clock,
            HSClockSource::APBClk => self.clock_control_config.apb_clock,
        })
    }

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = self.duty.unwrap() as u8;
        let sel_hstimer = self.clock_source.unwrap() == HSClockSource::APBClk;
        match self.number {
            Number::Timer0 => self.ledc.hstimer0_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer1 => self.ledc.hstimer1_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer2 => self.ledc.hstimer2_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer3 => self.ledc.hstimer3_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
        };
    }

    /// Update the timer in HW
    fn update_hw(&self) {
        // Nothing to do for HS timers
    }
}

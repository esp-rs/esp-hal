//! LEDC (LED PWM Controller) peripheral control
//!
//! Currently only supports fixed frequency output. Hardware fade support and
//! interrupts are not currently implemented. Low Speed channels are available.
//!
//! # LowSpeed Example:
//! The following will configure the Low Speed Channel0 to 24kHz output with
//! 10% duty using the ABPClock ```
//! let mut ledc = LEDC::new(&clock_control, &mut
//! system.peripheral_clock_control);
//!
//! ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
//! let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
//! lstimer0
//! .configure(timer::config::Config {
//!            duty: timer::config::Duty::Duty5Bit,
//!            clock_source: timer::LSClockSource::APBClk,
//!            frequency: 24u32.kHz(),
//!        })
//!        .unwrap();
//!
//! let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
//! channel0
//!     .configure(channel::config::Config {
//!         timer: &lstimer0,
//!         duty: 0.1,
//!     })
//!     .unwrap();
//! ```
//! # TODO
//! - Hardware fade support
//! - Interrupts

use channel::Channel;
use timer::Timer;

use self::timer::TimerSpeed;
use crate::{
    clock::Clocks,
    gpio::OutputPin,
    system::{Peripheral, PeripheralClockControl},
};

pub mod channel;
pub mod timer;

/// Global slow clock source
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSGlobalClkSource {
    APBClk,
}

/// LEDC (LED PWM Controller)
pub struct LEDC<'a> {
    ledc: &'a crate::pac::ledc::RegisterBlock,
    clock_control_config: &'a Clocks,
}

/// Used to specify LowSpeed Timer/Channel
pub struct LowSpeed {}

pub trait Speed {}


impl Speed for LowSpeed {}

impl<'a> LEDC<'a> {
    /// Return a new LEDC
    pub fn new(clock_control_config: &'a Clocks, system: &mut PeripheralClockControl) -> Self {
        system.enable(Peripheral::Ledc);

        let ledc = unsafe { &*crate::pac::LEDC::ptr() };
        LEDC {
            ledc,
            clock_control_config,
        }
    }

    /// Set global slow clock source
    pub fn set_global_slow_clock(&mut self, _clock_source: LSGlobalClkSource) {
        match _clock_source {
            LSGlobalClkSource::APBClk => {
                self.ledc.conf.write(|w| unsafe { w.apb_clk_sel().bits(1) })
            } /* LSGlobalClkSource::XTALClk => {
               *     self.ledc.conf.write(|w| unsafe { w.apb_clk_sel().bits(3) })
               * } */
        }
        self.ledc.timer0_conf.modify(|_, w| w.para_up().set_bit());
    }

    /// Return a new timer
    pub fn get_timer<S: TimerSpeed>(&self, number: timer::Number) -> Timer<S> {
        Timer::new(self.ledc, self.clock_control_config, number)
    }

    /// Return a new channel
    pub fn get_channel<S: TimerSpeed, O: OutputPin>(
        &self,
        number: channel::Number,
        output_pin: O,
    ) -> Channel<S, O> {
        Channel::new(number, output_pin)
    }
}

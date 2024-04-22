//! # LEDC (LED PWM Controller) peripheral control
//!
//! Currently only supports fixed-frequency output. Interrupts are not currently
//! implemented. High Speed channels are available for the ESP32 only, while Low
//! Speed channels are available for all supported chips.
//!
//! # LowSpeed Example:
//!
//! The following will configure the Low Speed Channel0 to 24kHz output with
//! 10% duty using the ABPClock
//!
//! ```no_run
//! let mut ledc = Ledc::new(peripherals.LEDC, &clock_control);
//! ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
//!
//! let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
//! lstimer0
//!     .configure(timer::config::Config {
//!         duty: timer::config::Duty::Duty5Bit,
//!         clock_source: timer::LSClockSource::APBClk,
//!         frequency: 24.kHz(),
//!     })
//!     .unwrap();
//!
//! let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
//! channel0
//!     .configure(channel::config::Config {
//!         timer: &lstimer0,
//!         duty: 10,
//!     })
//!     .unwrap();
//! ```
//!
//! # HighSpeed Example (ESP32 only):
//!
//! The following will configure the High Speed Channel0 to 24kHz output with
//! 10% duty using the ABPClock
//!
//! ```no_run
//! let ledc = Ledc::new(peripherals.LEDC, &clock_control);
//!
//! let mut hstimer0 = ledc.get_timer::<HighSpeed>(timer::Number::Timer0);
//! hstimer0
//!     .configure(timer::config::Config {
//!         duty: timer::config::Duty::Duty5Bit,
//!         clock_source: timer::HSClockSource::APBClk,
//!         frequency: 24.kHz(),
//!     })
//!     .unwrap();
//!
//! let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
//! channel0
//!     .configure(channel::config::Config {
//!         timer: &hstimer0,
//!         duty: 10,
//!     })
//!     .unwrap();
//! ```
//!
//! # Unsupported
//! - Source clock selection
//! - Interrupts

use self::{
    channel::Channel,
    timer::{Timer, TimerSpeed},
};
use crate::{
    clock::Clocks,
    gpio::OutputPin,
    peripheral::{Peripheral, PeripheralRef},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

pub mod channel;
pub mod timer;

/// Global slow clock source
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSGlobalClkSource {
    APBClk,
}

/// LEDC (LED PWM Controller)
pub struct Ledc<'d> {
    _instance: PeripheralRef<'d, crate::peripherals::LEDC>,
    ledc: &'d crate::peripherals::ledc::RegisterBlock,
    clock_control_config: &'d Clocks<'d>,
}

#[cfg(esp32)]
/// Used to specify HighSpeed Timer/Channel
pub struct HighSpeed {}

/// Used to specify LowSpeed Timer/Channel
pub struct LowSpeed {}

pub trait Speed {
    const IS_HS: bool;
}

#[cfg(esp32)]
impl Speed for HighSpeed {
    const IS_HS: bool = true;
}

impl Speed for LowSpeed {
    const IS_HS: bool = false;
}

impl<'d> Ledc<'d> {
    /// Return a new LEDC
    pub fn new(
        _instance: impl Peripheral<P = crate::peripherals::LEDC> + 'd,
        clock_control_config: &'d Clocks,
    ) -> Self {
        crate::into_ref!(_instance);
        PeripheralClockControl::enable(PeripheralEnable::Ledc);

        let ledc = unsafe { &*crate::peripherals::LEDC::ptr() };
        Ledc {
            _instance,
            ledc,
            clock_control_config,
        }
    }

    /// Set global slow clock source
    #[cfg(esp32)]
    pub fn set_global_slow_clock(&mut self, _clock_source: LSGlobalClkSource) {
        self.ledc.conf().write(|w| w.apb_clk_sel().set_bit());
        self.ledc
            .lstimer(0)
            .conf()
            .modify(|_, w| w.para_up().set_bit());
    }

    #[cfg(not(esp32))]
    /// Set global slow clock source
    pub fn set_global_slow_clock(&mut self, clock_source: LSGlobalClkSource) {
        #[cfg(any(esp32c6, esp32h2))]
        let pcr = unsafe { &*crate::peripherals::PCR::ptr() };

        #[cfg(any(esp32c6, esp32h2))]
        pcr.ledc_sclk_conf().write(|w| w.ledc_sclk_en().set_bit());

        match clock_source {
            LSGlobalClkSource::APBClk => {
                #[cfg(not(any(esp32c6, esp32h2)))]
                self.ledc
                    .conf()
                    .write(|w| unsafe { w.apb_clk_sel().bits(1) });
                #[cfg(esp32c6)]
                pcr.ledc_sclk_conf()
                    .write(|w| unsafe { w.ledc_sclk_sel().bits(1) });
                #[cfg(esp32h2)]
                pcr.ledc_sclk_conf()
                    .write(|w| unsafe { w.ledc_sclk_sel().bits(0) });
            }
        }
        self.ledc
            .timer(0)
            .conf()
            .modify(|_, w| w.para_up().set_bit());
    }

    /// Return a new timer
    pub fn get_timer<S: TimerSpeed>(&self, number: timer::Number) -> Timer<S> {
        Timer::new(self.ledc, self.clock_control_config, number)
    }

    /// Return a new channel
    pub fn get_channel<S: TimerSpeed, O: OutputPin>(
        &self,
        number: channel::Number,
        output_pin: impl Peripheral<P = O> + 'd,
    ) -> Channel<S, O> {
        Channel::new(number, output_pin)
    }
}

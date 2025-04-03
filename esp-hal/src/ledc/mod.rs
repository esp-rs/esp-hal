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
//! ## Configuration
//! Currently only supports fixed-frequency output. High Speed channels are
//! available for the ESP32 only, while Low Speed channels are available for all
//! supported chips.
//!
//! ## Examples
//!
//! ### Low Speed Channel
//!
//! The following example will configure the Low Speed Channel0 to 24kHz output
//! with 10% duty using the ABPClock and turn on LED with the option to change
//! LED intensity depending on `duty` value. Possible values (`u32`) are in
//! range 0..100.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::ledc::Ledc;
//! # use esp_hal::ledc::LSGlobalClkSource;
//! # use esp_hal::ledc::timer::{self, TimerIFace};
//! # use esp_hal::ledc::LowSpeed;
//! # use esp_hal::ledc::channel::{self, ChannelIFace};
//! # let led = peripherals.GPIO0;
//!
//! let mut ledc = Ledc::new(peripherals.LEDC);
//! ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
//!
//! let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
//! lstimer0
//!     .configure(timer::config::Config {
//!         duty: timer::config::Duty::Duty5Bit,
//!         clock_source: timer::LSClockSource::APBClk,
//!         frequency: Rate::from_khz(24),
//!     })?;
//!
//! let mut channel0 = ledc.channel(channel::Number::Channel0, led);
//! channel0
//!     .configure(channel::config::Config {
//!         timer: &lstimer0,
//!         duty_pct: 10,
//!         pin_config: channel::config::PinConfig::PushPull,
//!     })?;
//!
//! loop {
//!     // Set up a breathing LED: fade from off to on over a second, then
//!     // from on back off over the next second.  Then loop.
//!     channel0.start_duty_fade(0, 100, 1000)?;
//!     while channel0.is_duty_fade_running() {}
//!     channel0.start_duty_fade(100, 0, 1000)?;
//!     while channel0.is_duty_fade_running() {}
//! }
//! # }
//! ```
//! 
//! ## Implementation State
//! - Source clock selection is not supported
//! - Interrupts are not supported

use self::{
    channel::Channel,
    timer::{Timer, TimerSpeed},
};
use crate::{
    gpio::interconnect::PeripheralOutput,
    pac,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LEDC,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

pub mod channel;
pub mod timer;

/// Global slow clock source
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSGlobalClkSource {
    /// APB clock.
    APBClk,
}

/// LEDC (LED PWM Controller)
pub struct Ledc<'d> {
    _instance: PeripheralRef<'d, LEDC>,
    ledc: &'d pac::ledc::RegisterBlock,
}

#[cfg(esp32)]
#[derive(Clone, Copy)]
/// Used to specify HighSpeed Timer/Channel
pub struct HighSpeed {}

#[derive(Clone, Copy)]
/// Used to specify LowSpeed Timer/Channel
pub struct LowSpeed {}

/// Trait representing the speed mode of a clock or peripheral.
pub trait Speed {
    /// Boolean constant indicating whether the speed is high-speed.
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
    pub fn new(_instance: impl Peripheral<P = LEDC> + 'd) -> Self {
        crate::into_ref!(_instance);

        if PeripheralClockControl::enable(PeripheralEnable::Ledc) {
            PeripheralClockControl::reset(PeripheralEnable::Ledc);
        }

        let ledc = LEDC::regs();
        Ledc { _instance, ledc }
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
    pub fn timer<S: TimerSpeed>(&self, number: timer::Number) -> Timer<'d, S> {
        Timer::new(self.ledc, number)
    }

    /// Return a new channel
    pub fn channel<S: TimerSpeed>(
        &self,
        number: channel::Number,
        output_pin: impl PeripheralOutput<'d>,
    ) -> Channel<'d, S> {
        Channel::new(number, output_pin)
    }
}

//! # LED Controller (LEDC)
//!
//! ## Overview
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
//! ### Low Speed Channel
//! The following will configure the Low Speed Channel0 to 24kHz output with
//! 10% duty using the ABPClock
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::ledc::Ledc;
//! # use esp_hal::ledc::LSGlobalClkSource;
//! # use esp_hal::ledc::timer;
//! # use esp_hal::ledc::LowSpeed;
//! # use esp_hal::ledc::channel;
//! # use esp_hal::gpio::Io;
//! # use crate::esp_hal::prelude::_esp_hal_ledc_timer_TimerIFace;
//! # use crate::esp_hal::prelude::_fugit_RateExtU32;
//! # use crate::esp_hal::prelude::_esp_hal_ledc_channel_ChannelIFace;
//!
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let led = io.pins.gpio0;
//!
//! let mut ledc = Ledc::new(peripherals.LEDC, &clocks);
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
//!         duty_pct: 10,
//!         pin_config: channel::config::PinConfig::PushPull,
//!     })
//!     .unwrap();
//! # }
//! ```
//! 
//! ## Implementation State
//! - Source clock selection is not supported
//! - Interrupts are not supported

#![allow(missing_docs)] // TODO: Remove when able

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
        clock_control_config: &'d Clocks<'d>,
    ) -> Self {
        crate::into_ref!(_instance);

        PeripheralClockControl::reset(PeripheralEnable::Ledc);
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
    pub fn get_timer<S: TimerSpeed>(&self, number: timer::Number) -> Timer<'d, S> {
        Timer::new(self.ledc, self.clock_control_config, number)
    }

    /// Return a new channel
    pub fn get_channel<S: TimerSpeed, O: OutputPin>(
        &self,
        number: channel::Number,
        output_pin: impl Peripheral<P = O> + 'd,
    ) -> Channel<'d, S, O> {
        Channel::new(number, output_pin)
    }
}

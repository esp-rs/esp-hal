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
//! with 10% duty using the ABPClock.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::ledc::Ledc;
//! # use esp_hal::ledc::LSGlobalClkSource;
//! # use esp_hal::ledc::timer;
//! # use esp_hal::ledc::LowSpeed;
//! # use esp_hal::ledc::channel;
//! # use esp_hal::gpio::Io;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let led = io.pins.gpio0;
//!
//! let mut ledc = Ledc::new(peripherals.LEDC);
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

use self::{channel::Channel, timer::TimerSpeed};
use crate::{
    gpio::OutputPin,
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{ledc::RegisterBlock, Interrupt},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    InterruptConfigurable,
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
    _instance: PeripheralRef<'d, crate::peripherals::LEDC>,
}

#[cfg(esp32)]
/// Used to specify HighSpeed Timer/Channel
pub struct HighSpeed {}

/// Used to specify LowSpeed Timer/Channel
pub struct LowSpeed {}

/// Trait representing the speed mode of a clock or peripheral.
pub trait Speed: Send + Sync {
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
    pub fn new(_instance: impl Peripheral<P = crate::peripherals::LEDC> + 'd) -> Self {
        crate::into_ref!(_instance);

        PeripheralClockControl::reset(PeripheralEnable::Ledc);
        PeripheralClockControl::enable(PeripheralEnable::Ledc);

        Ledc { _instance }
    }

    fn register_block() -> &'static RegisterBlock {
        unsafe { &*crate::peripherals::LEDC::ptr() }
    }

    /// Set global slow clock source
    #[cfg(esp32)]
    pub fn set_global_slow_clock(&mut self, _clock_source: LSGlobalClkSource) {
        Self::register_block()
            .conf()
            .write(|w| w.apb_clk_sel().set_bit());
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
                Self::register_block()
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
    }

    /// Get the global slow clock source
    /// Returns `Some(LSGlobalClkSource)` if the source is APB clock, otherwise
    /// `None`, which means the source clock is either unconfigured or not
    /// supported.
    pub fn get_global_slow_clock() -> Option<LSGlobalClkSource> {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                match Self::register_block().conf().read().apb_clk_sel().bit() {
                    true => Some(LSGlobalClkSource::APBClk),
                    false => None,
                }
            }else if #[cfg(esp32c6)] {
                let pcr = unsafe { &*crate::peripherals::PCR::ptr() };
                match pcr.ledc_sclk_conf().read().ledc_sclk_sel().bits() {
                    1 => Some(LSGlobalClkSource::APBClk),
                    _ => None,
                }
            }else if #[cfg(esp32h2)] {
                let pcr = unsafe { &*crate::peripherals::PCR::ptr() };
                match pcr.ledc_sclk_conf().read().ledc_sclk_sel().bits() {
                    0 => Some(LSGlobalClkSource::APBClk),
                    _ => None,
                }
            }else {
                match Self::register_block().conf().read().apb_clk_sel().bits() {
                    1 => Some(LSGlobalClkSource::APBClk),
                    _ => None,
                }
            }
        }
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

impl<'d> crate::private::Sealed for Ledc<'d> {}

impl<'d> InterruptConfigurable for Ledc<'d> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            interrupt::bind_interrupt(Interrupt::LEDC, handler.handler());
            interrupt::enable(Interrupt::LEDC, handler.priority()).unwrap();
        }
    }
}

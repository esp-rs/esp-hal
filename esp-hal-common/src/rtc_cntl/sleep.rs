//! # RTC Control Sleep Module
//!
//! ## Overview
//! The `sleep` module in the `RTC CNTL (Real-Time Clock Control)` driver
//! provides functionality to manage sleep and wakeup sources for `ESP` chips.
//! The `RTC_CNTL` is responsible for controlling the power and sleep behavior
//! of the chip.
//!
//! The `sleep` module allows configuring various wakeup sources and setting up
//! the sleep behavior based on those sources. The supported wakeup sources
//! include:
//!    * `GPIO` pins - light sleep only
//!    * timers
//!    * `SDIO (Secure Digital Input/Output) - light sleep only`
//!    * `MAC (Media Access Control)` wake - light sleep only
//!    * `UART0` - light sleep only
//!    * `UART1` - light sleep only
//!    * `touch`
//!    * `ULP (Ultra-Low Power)` wake
//!    * `BT (Bluetooth) wake` - light sleep only

use core::{cell::RefCell, time::Duration};

#[cfg(esp32c3)]
use crate::gpio::RTCPinWithResistors;
use crate::{gpio::RTCPin, Rtc};

#[cfg_attr(esp32, path = "rtc/esp32_sleep.rs")]
#[cfg_attr(esp32s3, path = "rtc/esp32s3_sleep.rs")]
#[cfg_attr(esp32c3, path = "rtc/esp32c3_sleep.rs")]
mod rtc_sleep;

pub use rtc_sleep::*;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum WakeupLevel {
    Low,
    #[default]
    High,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct TimerWakeupSource {
    duration: Duration,
}

impl TimerWakeupSource {
    pub fn new(duration: Duration) -> Self {
        Self { duration }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    NotRtcPin,
    TooManyWakeupSources,
}

#[allow(unused)]
#[derive(Debug)]
pub struct Ext0WakeupSource<'a, P: RTCPin> {
    pin: RefCell<&'a mut P>,
    level: WakeupLevel,
}

impl<'a, P: RTCPin> Ext0WakeupSource<'a, P> {
    pub fn new(pin: &'a mut P, level: WakeupLevel) -> Self {
        Self {
            pin: RefCell::new(pin),
            level,
        }
    }
}

#[allow(unused)]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [&'b mut dyn RTCPin]>,
    level: WakeupLevel,
}

impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    pub fn new(pins: &'a mut [&'b mut dyn RTCPin], level: WakeupLevel) -> Self {
        Self {
            pins: RefCell::new(pins),
            level,
        }
    }
}

/// RTC_IO wakeup source
///
/// RTC_IO wakeup allows configuring any combination of RTC_IO pins with
/// arbitrary wakeup levels to wake up the chip from sleep. This wakeup source
/// can be used to wake up from both light and deep sleep.
#[allow(unused)]
pub struct RtcioWakeupSource<'a, 'b> {
    #[cfg(xtensa)]
    pins: RefCell<&'a mut [(&'b mut dyn RTCPin, WakeupLevel)]>,
    #[cfg(esp32c3)]
    pins: RefCell<&'a mut [(&'b mut dyn RTCPinWithResistors, WakeupLevel)]>,
}

impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    #[cfg(xtensa)]
    pub fn new(pins: &'a mut [(&'b mut dyn RTCPin, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }

    #[cfg(esp32c3)]
    pub fn new(pins: &'a mut [(&'b mut dyn RTCPinWithResistors, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

bitfield::bitfield! {
    #[derive(Default, Clone, Copy)]
    pub struct WakeTriggers(u16);
    impl Debug;
    /// EXT0 GPIO wakeup
    pub ext0, set_ext0: 0;
    /// EXT1 GPIO wakeup
    pub ext1, set_ext1: 1;
    /// GPIO wakeup (light sleep only)
    pub gpio, set_gpio: 2;
    /// Timer wakeup
    pub timer, set_timer: 3;
    /// SDIO wakeup (light sleep only)
    pub sdio, set_sdio: 4;
    /// MAC wakeup (light sleep only)
    pub mac, set_mac: 5;
    /// UART0 wakeup (light sleep only)
    pub uart0, set_uart0: 6;
    /// UART1 wakeup (light sleep only)
    pub uart1, set_uart1: 7;
    /// Touch wakeup
    pub touch, set_touch: 8;
    /// ULP wakeup
    pub ulp, set_ulp: 9;
    /// BT wakeup (light sleep only)
    pub bt, set_bt: 10;
}

pub trait WakeSource {
    fn apply(&self, rtc: &Rtc, triggers: &mut WakeTriggers, sleep_config: &mut RtcSleepConfig);
}

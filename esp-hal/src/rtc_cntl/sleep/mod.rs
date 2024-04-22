//! # RTC Control Sleep Module
//!
//! ## Overview
//!
//! The `sleep` module in the `RTC CNTL (Real-Time Control)` driver
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

use core::cell::RefCell;
#[cfg(any(esp32, esp32c3, esp32s3, esp32c6))]
use core::time::Duration;

#[cfg(any(esp32, esp32s3))]
use crate::gpio::RtcPin as RtcIoWakeupPinType;
#[cfg(any(esp32c3, esp32c6))]
use crate::gpio::RtcPinWithResistors as RtcIoWakeupPinType;
use crate::rtc_cntl::Rtc;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32c3, path = "esp32c3.rs")]
#[cfg_attr(esp32c6, path = "esp32c6.rs")]
mod sleep_impl;

pub use sleep_impl::*;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum WakeupLevel {
    Low,
    #[default]
    High,
}

#[derive(Debug, Default, Clone, Copy)]
#[cfg(any(esp32, esp32c3, esp32s3, esp32c6))]
pub struct TimerWakeupSource {
    duration: Duration,
}

#[cfg(any(esp32, esp32c3, esp32s3, esp32c6))]
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

#[derive(Debug)]
#[cfg(any(esp32, esp32s3))]
pub struct Ext0WakeupSource<'a, P: RtcIoWakeupPinType> {
    pin: RefCell<&'a mut P>,
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s3))]
impl<'a, P: RtcIoWakeupPinType> Ext0WakeupSource<'a, P> {
    pub fn new(pin: &'a mut P, level: WakeupLevel) -> Self {
        Self {
            pin: RefCell::new(pin),
            level,
        }
    }
}

#[cfg(any(esp32, esp32s3))]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [&'b mut dyn RtcIoWakeupPinType]>,
    level: WakeupLevel,
}

#[cfg(any(esp32, esp32s3))]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    pub fn new(pins: &'a mut [&'b mut dyn RtcIoWakeupPinType], level: WakeupLevel) -> Self {
        Self {
            pins: RefCell::new(pins),
            level,
        }
    }
}

#[cfg(esp32c6)]
pub struct Ext1WakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(esp32c6)]
impl<'a, 'b> Ext1WakeupSource<'a, 'b> {
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

/// RTC_IO wakeup source
///
/// RTC_IO wakeup allows configuring any combination of RTC_IO pins with
/// arbitrary wakeup levels to wake up the chip from sleep. This wakeup source
/// can be used to wake up from both light and deep sleep.
#[cfg(any(esp32c3, esp32s3))]
pub struct RtcioWakeupSource<'a, 'b> {
    pins: RefCell<&'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]>,
}

#[cfg(any(esp32c3, esp32s3))]
impl<'a, 'b> RtcioWakeupSource<'a, 'b> {
    pub fn new(pins: &'a mut [(&'b mut dyn RtcIoWakeupPinType, WakeupLevel)]) -> Self {
        Self {
            pins: RefCell::new(pins),
        }
    }
}

#[cfg(not(pmu))]
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

#[cfg(pmu)]
bitfield::bitfield! {
    #[derive(Default, Clone, Copy)]
    pub struct WakeTriggers(u16);
    impl Debug;

    /// EXT0 GPIO wakeup
    pub ext0, set_ext0: 0;
    /// EXT1 GPIO wakeup
    pub ext1, set_ext1: 1;
    /// GPIO wakeup
    pub gpio, set_gpio: 2;
    /// WiFi beacon wakeup
    pub wifi_beacon, set_wifi_beacon: 3;
    /// Timer wakeup
    pub timer, set_timer: 4;
    /// WiFi SoC wakeup
    pub wifi_soc, set_wifi_soc: 5;
    /// UART0 wakeup
    pub uart0, set_uart0: 6;
    /// UART1 wakeup
    pub uart1, set_uart1: 7;
    /// SDIO wakeup
    pub sdio, set_sdio: 8;
    /// BT wakeup
    pub bt, set_bt: 10;
    /// LP core wakeup
    pub lp_core, set_lp_core: 11;
    /// USB wakeup
    pub usb, set_usb: 14;
}

pub trait WakeSource {
    fn apply(&self, rtc: &Rtc, triggers: &mut WakeTriggers, sleep_config: &mut RtcSleepConfig);
}

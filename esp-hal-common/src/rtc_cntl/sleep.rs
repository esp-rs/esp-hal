use core::{cell::RefCell, time::Duration};

use crate::{
    gpio::{Pin, RTCPin},
    Rtc,
};

#[cfg_attr(esp32, path = "rtc/esp32_sleep.rs")]
#[cfg_attr(esp32s3, path = "rtc/esp32s3_sleep.rs")]
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

#[derive(Debug, Clone, Copy)]
pub enum Error {
    NotRtcPin,
    TooManyWakeupSources,
}

#[allow(unused)]
#[derive(Debug)]
pub struct Ext0WakeupSource<'a, P: RTCPin + Pin> {
    pin: RefCell<&'a mut P>,
    level: WakeupLevel,
}

impl<'a, P: RTCPin + Pin> Ext0WakeupSource<'a, P> {
    pub fn new(pin: &'a mut P, level: WakeupLevel) -> Self {
        Self {
            pin: RefCell::new(pin),
            level,
        }
    }
}

pub struct Ext1WakeupSource<'a> {
    pins: RefCell<&'a mut [&'a mut dyn RTCPin]>,
    level: WakeupLevel,
}

impl<'a> Ext1WakeupSource<'a> {
    pub fn new(pins: &'a mut [&'a mut dyn RTCPin], level: WakeupLevel) -> Self {
        Self {
            pins: RefCell::new(pins),
            level,
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

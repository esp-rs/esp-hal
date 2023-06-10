use core::time::Duration;

use crate::{gpio::Pin, Rtc};

#[cfg_attr(esp32, path = "rtc/esp32_sleep.rs")]
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
// TODO: restrict to RTCPin as well (when its implemented on RTC pins)
pub struct Ext0WakeupSource<'a, P: Pin> {
    pin: &'a mut P,
    level: WakeupLevel,
}

impl<'a, P: Pin> Ext0WakeupSource<'a, P> {
    pub fn new(pin: &'a mut P, level: WakeupLevel) -> Self {
        Self { pin, level }
    }
    // TODO: esp32 only! - needs to be re-factored (should be in RTCPin impl)
    fn to_rtc_pin(&self) -> Result<u8, Error> {
        match self.pin.number() {
            0 => Ok(11),
            2 => Ok(12),
            4 => Ok(10),
            12 => Ok(15),
            13 => Ok(14),
            14 => Ok(16),
            15 => Ok(13),
            25 => Ok(6),
            26 => Ok(7),
            27 => Ok(17),
            32 => Ok(9),
            33 => Ok(8),
            34 => Ok(4),
            35 => Ok(5),
            36 => Ok(0),
            37 => Ok(1),
            38 => Ok(2),
            39 => Ok(3),
            _ => Err(Error::NotRtcPin),
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

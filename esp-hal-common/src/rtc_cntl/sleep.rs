use core::time::Duration;

use crate::{gpio::Pin, Rtc};

#[cfg_attr(esp32, path = "rtc/esp32_sleep.rs")]
mod rtc_sleep;
#[cfg(esp32)]
use esp32 as pac;
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
    // TODO: esp32 only! - needs to be re-factored
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
// RTCIO_GPIO0_CHANNEL,    //GPIO0
// RTCIO_GPIO2_CHANNEL,    //GPIO2
// RTCIO_GPIO4_CHANNEL,    //GPIO4
// RTCIO_GPIO12_CHANNEL,   //GPIO12
// RTCIO_GPIO13_CHANNEL,   //GPIO13
// RTCIO_GPIO14_CHANNEL,   //GPIO14
// RTCIO_GPIO15_CHANNEL,   //GPIO15
// RTCIO_GPIO25_CHANNEL,   //GPIO25
// RTCIO_GPIO26_CHANNEL,   //GPIO26
// RTCIO_GPIO27_CHANNEL,   //GPIO27
// RTCIO_GPIO32_CHANNEL,   //GPIO32
// RTCIO_GPIO33_CHANNEL,   //GPIO33
// RTCIO_GPIO34_CHANNEL,   //GPIO34
// RTCIO_GPIO35_CHANNEL,   //GPIO35
// RTCIO_GPIO36_CHANNEL,   //GPIO36
// RTCIO_GPIO37_CHANNEL,   //GPIO37
// RTCIO_GPIO38_CHANNEL,   //GPIO38
// RTCIO_GPIO39_CHANNEL,   //GPIO39
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
    fn prepare(&self, rtc: &Rtc, triggers: &mut WakeTriggers);
}

// non-alloc version?
extern crate alloc;
pub struct Sleep<'a> {
    sleep_config: RtcSleepConfig,
    wake_sources: alloc::vec::Vec<&'a dyn WakeSource>,
}

impl<'a> Default for Sleep<'a> {
    fn default() -> Self {
        Self {
            sleep_config: Default::default(),
            wake_sources: Default::default(),
        }
    }
}
impl<'a> Sleep<'a> {
    pub fn new() -> Self {
        Self {
            ..Default::default()
        }
    }

    pub fn deep() -> Self {
        Self {
            sleep_config: RtcSleepConfig::deep(),
            ..Default::default()
        }
    }

    pub fn add_wakeup_source(&mut self, wake_source: &'a impl WakeSource) {
        self.wake_sources.push(wake_source)
    }

    pub fn sleep(&self, rtc: &mut Rtc, delay: &mut crate::Delay) {
        self.sleep_config.apply(rtc);
        let mut wakeup_triggers = WakeTriggers::default();
        for wake_source in &self.wake_sources {
            wake_source.prepare(rtc, &mut wakeup_triggers)
        }
        use embedded_hal::blocking::delay::DelayMs;
        delay.delay_ms(100u32);
        unsafe {
            let rtc_cntl = &*pac::RTC_CNTL::ptr();

            rtc_cntl
                .reset_state
                .modify(|_, w| w.procpu_stat_vector_sel().set_bit());

            // set bits for what can wake us up
            rtc_cntl
                .wakeup_state
                .modify(|_, w| w.wakeup_ena().bits(wakeup_triggers.0.into()));

            // TODO: remove this!
            // esp_reg_dump::rtc_cntl::dump_all();

            rtc_cntl
                .state0
                .write(|w| w.sleep_en().set_bit().slp_wakeup().set_bit());
        }
    }
}

use core::time::Duration;

use crate::{
    gpio::{Pin, RTCPin},
    Rtc,
};

#[cfg_attr(esp32, path = "rtc/esp32_sleep.rs")]
mod rtc_sleep;
#[cfg(esp32)]
use esp32 as pac;
pub use rtc_sleep::*;

#[derive(Debug, Default, Clone, Copy)]
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

#[allow(unused)]
#[derive(Debug)]
pub struct Ext0WakeupSource<'a, P: Pin + RTCPin> {
    pin: &'a mut P,
    level: WakeupLevel,
}
impl<'a, P: Pin + RTCPin> Ext0WakeupSource<'a, P> {
    pub fn new(pin: &'a mut P, level: WakeupLevel) -> Self {
        Self { pin, level }
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
    fn prepare(&self, rtc: &Rtc, triggers: &mut WakeTriggers);
}

// non-alloc version?
extern crate alloc;
#[derive(Debug)]
pub struct Sleep<'a, W: WakeSource> {
    sleep_config: RtcSleepConfig,
    wake_sources: alloc::vec::Vec<&'a W>,
}

impl<'a, W: WakeSource> Default for Sleep<'a, W> {
    fn default() -> Self {
        Self {
            sleep_config: Default::default(),
            wake_sources: Default::default(),
        }
    }
}
impl<'a, W: WakeSource> Sleep<'a, W> {
    pub fn new() -> Sleep<'a, W> {
        Self {
            ..Default::default()
        }
    }

    pub fn deep() -> Sleep<'a, W> {
        Self {
            sleep_config: RtcSleepConfig::deep(),
            ..Default::default()
        }
    }

    pub fn add_wakeup_source(&mut self, wake_source: &'a W) {
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

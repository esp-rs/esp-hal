use core::time::Duration;

use crate::Rtc;

#[cfg_attr(esp32, path = "rtc/esp32_sleep.rs")]
mod rtc_sleep;
pub use rtc_sleep::*;

#[cfg(esp32)]
use esp32 as pac;

#[derive(Debug, Default, Clone, Copy)]
struct TimerWakeSource {
    duration: Duration,
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

trait WakeSource {
    fn prepare(&self, rtc: &Rtc);
}

#[derive(Debug, Default)]
pub struct Sleep {
    sleep_config: RtcSleepConfig,
    wakeup_triggers: WakeTriggers,
    timer_wake: Option<TimerWakeSource>,
}

impl Sleep {
    pub fn new() -> Sleep {
        Self {
            ..Default::default()
        }
    }

    pub fn deep() -> Sleep {
        Self {
            sleep_config: RtcSleepConfig::deep(),
            ..Default::default()
        }
    }

    pub fn timer(&mut self, duration: Duration) {
        self.timer_wake = Some(TimerWakeSource { duration });
        self.wakeup_triggers.set_timer(true);
    }

    pub fn sleep(&self, rtc: &mut Rtc, delay: &mut crate::Delay) {
        self.sleep_config.apply(rtc);
        if let Some(timer) = &self.timer_wake {
            timer.prepare(rtc);
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
                .modify(|_, w| w.wakeup_ena().bits(self.wakeup_triggers.0.into()));

            // TODO: remove this!
            // esp_reg_dump::rtc_cntl::dump_all();

            rtc_cntl
                .state0
                .write(|w| w.sleep_en().set_bit().slp_wakeup().set_bit());
        }
    }
}

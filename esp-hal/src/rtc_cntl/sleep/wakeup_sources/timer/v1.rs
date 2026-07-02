use super::TimerWakeupSource;
use crate::{
    peripherals::LPWR,
    rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers},
};

impl WakeSource for TimerWakeupSource {
    fn apply(&self, rtc: &Rtc<'_>, triggers: &mut WakeTriggers, sleep_config: &mut RtcSleepConfig) {
        // don't power down RTC peripherals
        sleep_config.set_rtc_peri_pd_en(false);

        triggers.set_timer(true);

        let ticks = crate::clock::us_to_rtc_ticks(self.duration.as_micros());
        let now = rtc.time_since_boot_raw();
        let time_in_ticks = now.wrapping_add(ticks);

        unsafe {
            LPWR::regs()
                .int_clr()
                .write(|w| w.main_timer().clear_bit_by_one());

            LPWR::regs()
                .slp_timer0()
                .write(|w| w.slp_val_lo().bits((time_in_ticks & 0xffffffff) as u32));

            LPWR::regs().slp_timer1().write(|w| {
                w.slp_val_hi().bits(((time_in_ticks >> 32) & 0xffff) as u16);
                w.main_timer_alarm_en().set_bit()
            });
        }
    }
}

use super::TimerWakeupSource;
use crate::{
    peripherals::LP_TIMER,
    rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers},
};

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);

        let ticks = crate::clock::us_to_rtc_ticks(self.duration.as_micros());
        let now = rtc.time_since_boot_raw();
        let time_in_ticks = now.wrapping_add(ticks);

        unsafe {
            LP_TIMER::regs().tar0_high().write(|w| {
                w.main_timer_tar_high0()
                    .bits(((time_in_ticks >> 32) & 0xffff) as u16)
            });
            LP_TIMER::regs().tar0_low().write(|w| {
                w.main_timer_tar_low0()
                    .bits((time_in_ticks & 0xffffffff) as u32)
            });
            LP_TIMER::regs()
                .int_clr()
                .write(|w| w.soc_wakeup().clear_bit_by_one());
            LP_TIMER::regs()
                .tar0_high()
                .modify(|_, w| w.main_timer_tar_en0().set_bit());
        }
    }
}

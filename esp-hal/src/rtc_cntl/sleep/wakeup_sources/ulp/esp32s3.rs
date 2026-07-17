use super::UlpWakeupSource;
use crate::rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource};

impl WakeSource for UlpWakeupSource {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        if self.wake_on_interrupt {
            triggers.insert(WakeupSource::Ulp);
            triggers.insert(WakeupSource::UlpRiscv);
        }
        if self.wake_on_trap {
            triggers.insert(WakeupSource::UlpRiscvTrap);
        }

        if self.clear_interrupts_on_sleep {
            self.clear_interrupts();
        }

        // This one needs to be false to keep the ULP timer and ULP GPIO happy!
        // Possibly relevant issue: https://github.com/espressif/esp-idf/issues/10595
        sleep_config.set_rtc_peri_pd_en(false);
    }
}

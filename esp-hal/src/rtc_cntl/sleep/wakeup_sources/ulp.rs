use crate::rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource};

/// RISC-V ULP wakeup source
///
/// Wake up from ULP software interrupt, and/or ULP-RISCV Trap condition.
/// Both of these triggers are enabled by default.
/// This source will clear any outstanding software interrupts prior to entering sleep, by default.
///
/// This wakeup source can be used to wake up from both light and deep sleep.
pub struct UlpWakeupSource {
    wake_on_interrupt: bool,
    wake_on_trap: bool,
    clear_interrupts_on_sleep: bool,
}

impl UlpWakeupSource {
    /// Create a new instance of `WakeFromUlpWakeupSource`
    pub const fn new() -> Self {
        Self {
            wake_on_interrupt: true,
            wake_on_trap: true,
            clear_interrupts_on_sleep: true,
        }
    }

    /// Enable wakeup triggered by software interrupt from ULP-RISCV
    pub fn set_wake_on_interrupt(mut self, value: bool) -> Self {
        self.wake_on_interrupt = value;
        self
    }

    /// Enable wakeup triggered by ULP-RISCV Trap
    pub fn set_wake_on_trap(mut self, value: bool) -> Self {
        self.wake_on_trap = value;
        self
    }

    /// Enable clearing of latched wake-up interrupts prior to entering sleep
    pub fn set_clear_interrupts_on_sleep(mut self, value: bool) -> Self {
        self.clear_interrupts_on_sleep = value;
        self
    }

    /// Clears the wake-up interrupts
    pub fn clear_interrupts(&self) {
        crate::peripherals::LPWR::regs().int_clr().write(|w| {
            w.cocpu_trap().clear_bit_by_one();
            w.cocpu().clear_bit_by_one();
            w.ulp_cp().clear_bit_by_one()
        });
    }
}

impl Default for UlpWakeupSource {
    fn default() -> Self {
        Self::new()
    }
}

impl WakeSource for UlpWakeupSource {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        if self.wake_on_interrupt {
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

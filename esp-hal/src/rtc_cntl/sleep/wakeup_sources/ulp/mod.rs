/// ULP wakeup source
///
/// Wake up from ULP software interrupt, and/or ULP-RISCV Trap condition.
/// Both of these triggers are enabled by default.
/// This source will clear any outstanding software interrupts prior to entering sleep, by default.
///
/// S2 supports the following triggers (Refer to ESP32-S2 Technical Reference Manual, Table 9.4-3.
/// Wakeup Source)
///  - ULP-FSM software interrupt (unsure if this ALSO supports ULP-RISCV software interrupt)
///  - ULP-RISCV Trap
///
/// S3 supports the following triggers (Refer to ESP32-S3 Technical Reference Manual, Table 10.4-3.
/// Wakeup Source)
///  - ULP-FSM software interrupt and ULP-RISCV software interrupt
///  - ULP-RISCV Trap
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

    /// Enable wakeup triggered by software interrupt from ULP-FSM or ULP-RISCV
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

#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
mod implementation;

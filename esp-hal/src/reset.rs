//! # Hardware and Software Reset
//!
//! ## Overview
//! Espressif chips provide four types of reset that occur at different levels,
//! namely CPU Reset, Core Reset, System Reset, and Chip Reset. All reset types
//! mentioned above (except Chip Reset) preserve the data stored in internal
//! memory.
//!
//! The Hardware and Software Reset module provides functions for
//! performing hardware and software resets and includes functions for
//! retrieving the reset reason and the wakeup cause after a reset.
//!
//! The module defines a set of sleep sources (`SleepSource`) that indicate the
//! source of the wakeup event. It also includes a set of flags (`WakeupReason`)
//! that represent different wakeup sources and enable/disable wakeup triggers
//! for specific events.

use crate::rtc_cntl::SocResetReason;

/// Source of the wakeup event
#[derive(Debug, Copy, Clone)]
pub enum SleepSource {
    /// In case of deep sleep, reset was not caused by exit from deep sleep
    Undefined = 0,
    /// Not a wakeup cause, used to disable all wakeup sources with
    /// esp_sleep_disable_wakeup_source
    All,
    /// Wakeup caused by external signal using RTC_IO
    Ext0,
    /// Wakeup caused by external signal using RTC_CNTL
    Ext1,
    /// Wakeup caused by timer
    Timer,
    /// Wakeup caused by touchpad
    TouchPad,
    /// Wakeup caused by ULP program
    Ulp,
    /// Wakeup caused by GPIO (light sleep only on ESP32, S2 and S3)
    Gpio,
    /// Wakeup caused by UART (light sleep only)
    Uart,
    /// Wakeup caused by WIFI (light sleep only)
    Wifi,
    /// Wakeup caused by COCPU int
    Cocpu,
    /// Wakeup caused by COCPU crash
    CocpuTrapTrig,
    /// Wakeup caused by BT (light sleep only)
    BT,
}

bitflags::bitflags! {
    #[allow(unused)]
    pub(crate) struct WakeupReason: u32 {
        const NoSleep         = 0;
        #[cfg(pm_support_ext0_wakeup)]
        /// EXT0 GPIO wakeup
        const ExtEvent0Trig   = 1 << 0;
        #[cfg(pm_support_ext1_wakeup)]
        /// EXT1 GPIO wakeup
        const ExtEvent1Trig   = 1 << 1;
        /// GPIO wakeup (light sleep only)
        const GpioTrigEn      = 1 << 2;
        #[cfg(not(any(esp32c6, esp32h2)))]
        /// Timer wakeup
        const TimerTrigEn     = 1 << 3;
        #[cfg(any(esp32c6, esp32h2))]
        /// Timer wakeup
        const TimerTrigEn     = 1 << 4;
        #[cfg(pm_support_wifi_wakeup)]
        /// MAC wakeup (light sleep only)
        const WifiTrigEn      = 1 << 5;
        /// UART0 wakeup (light sleep only)
        const Uart0TrigEn     = 1 << 6;
        /// UART1 wakeup (light sleep only)
        const Uart1TrigEn     = 1 << 7;
        #[cfg(pm_support_touch_sensor_wakeup)]
        /// Touch wakeup
        const TouchTrigEn     = 1 << 8;
        #[cfg(ulp_supported)]
        /// ULP wakeup
        const UlpTrigEn       = 1 << 9;
        #[cfg(pm_support_bt_wakeup)]
        /// BT wakeup (light sleep only)
        const BtTrigEn        = 1 << 10;
        #[cfg(riscv_coproc_supported)]
        const CocpuTrigEn     = 1 << 11;
        #[cfg(riscv_coproc_supported)]
        const CocpuTrapTrigEn = 1 << 13;
    }
}

/// Performs a software reset on the chip.
pub fn software_reset() {
    crate::rom::software_reset();
}

/// Performs a software reset on the CPU.
pub fn software_reset_cpu() {
    crate::rom::software_reset_cpu();
}

/// Retrieves the reason for the last reset as a SocResetReason enum value.
/// Returns `None` if the reset reason cannot be determined.
pub fn reset_reason() -> Option<SocResetReason> {
    crate::rtc_cntl::reset_reason(crate::Cpu::current())
}

/// Retrieves the cause of the last wakeup event as a SleepSource enum value.
pub fn wakeup_cause() -> SleepSource {
    crate::rtc_cntl::wakeup_cause()
}

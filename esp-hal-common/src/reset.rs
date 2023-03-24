use crate::rtc_cntl::SocResetReason;

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

#[allow(unused)]
pub(crate) enum WakeupReason {
    NoSleep           = 0,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
    /// EXT0 GPIO wakeup
    ExtEvent0Trig     = 1 << 0,
    #[cfg(not(any(esp32c2, esp32c3)))]
    /// EXT1 GPIO wakeup
    ExtEvent1Trig     = 1 << 1,
    /// GPIO wakeup (light sleep only)
    GpioTrigEn        = 1 << 2,
    /// Timer wakeup
    TimerTrigEn       = 1 << 3,
    #[cfg(not(any(esp32c2, esp32c3)))]
    /// SDIO wakeup (light sleep only)
    SdioTrigEn        = 1 << 4,
    /// MAC wakeup (light sleep only)
    WifiTrigEn        = 1 << 5,
    /// UART0 wakeup (light sleep only)
    Uart0TrigEn       = 1 << 6,
    /// UART1 wakeup (light sleep only)
    Uart1TrigEn       = 1 << 7,
    #[cfg(not(any(esp32c2, esp32c3)))]
    /// Touch wakeup
    TouchTrigEn       = 1 << 8,
    #[cfg(not(any(esp32c2, esp32c3)))]
    /// ULP wakeup
    UlpTrigEn         = 1 << 9,
    /// BT wakeup (light sleep only)
    BtTrigEn          = 1 << 10,
    #[cfg(any(esp32s2, esp32s3))]
    CocpuTrigEn       = 1 << 11,
    #[cfg(not(esp32))]
    Xtal32kDeadTrigEn = 1 << 12,
    #[cfg(any(esp32s2, esp32s3))]
    CocpuTrapTrigEn   = 1 << 13,
    #[cfg(not(esp32))]
    UsbTrigEn         = 1 << 14,
    #[cfg(esp32c3)]
    BrownoutDetTrigEn = 1 << 16,
}

pub fn software_reset() {
    unsafe { crate::rtc_cntl::software_reset() }
}
pub fn software_reset_cpu() {
    unsafe { crate::rtc_cntl::software_reset_cpu() }
}

pub fn get_reset_reason() -> Option<SocResetReason> {
    crate::rtc_cntl::get_reset_reason(crate::get_core())
}

pub fn get_wakeup_cause() -> SleepSource {
    crate::rtc_cntl::get_wakeup_cause()
}

use fugit::HertzU32;
use strum::FromRepr;

use crate::clock::Clock;

pub(crate) fn init() {
    todo!()
}

pub(crate) fn configure_clock() {
    todo!()
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

// spc/p4/include/soc/reset_reasons.h
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` or
    /// `ChipSuperWdt`, however that is not really compatible with Rust-style
    /// enums.
    ChipPowerOn = 0x01,
    /// Software resets the digital core by RTC_CNTL_SW_SYS_RST
    CoreSw = 0x03,
    /// Deep sleep reset the digital core
    CoreDeepSleep = 0x05,
    // PMU HP power down system reset
    SysPmuPwrDown = 0x05,
    // PMU HP power down CPU reset
    CpuPmuPwrDown = 0x06,
    /// HP watch dog resets system
    SysHpWdt = 0x07,
    /// LP watch dog resets system
    SysLpWdt = 0x09,
    /// HP watch dog resets digital core
    CoreHpWdt = 0x0B,
    /// Software resets CPU 0
    Cpu0Sw = 0x0C,
    /// LP watch dog resets digital core
    CpuLpWdt = 0x0D,
    /// VDD voltage is not stable and resets the digital core
    SysBrownOut = 0x0F,
    /// LP watch dog resets chip
    ChipLpWdt = 0x10,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt = 0x12,
    /// Glitch on clock resets the digital core and rtc module
    SysClkGlitch = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc = 0x14,
    /// USB JTAG resets the digital core
    CoreUsbJtag = 0x16,
    // USB Serial/JTAG controller's UART resets the digital core
    CoreUsbUart = 0x17,
    // Glitch on power resets the digital core
    CpuJtag = 0x18,
}

/// RTC SLOW_CLK frequency values
#[derive(Debug, Clone, Copy)]
pub(crate) enum RtcFastClock {}

impl Clock for RtcFastClock {
    fn frequency(&self) -> HertzU32 {
        todo!()
    }
}

/// RTC SLOW_CLK frequency values
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum RtcSlowClock {}

impl Clock for RtcSlowClock {
    fn frequency(&self) -> HertzU32 {
        todo!()
    }
}

/// RTC Watchdog Timer
pub struct RtcClock;

/// RTC Watchdog Timer driver
impl RtcClock {
    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        todo!()
    }
}

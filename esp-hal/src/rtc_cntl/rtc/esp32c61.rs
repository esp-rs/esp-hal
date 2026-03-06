use strum::FromRepr;

pub(crate) fn init() {}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

/// SOC Reset Reason (ESP32-C61).
///
/// Parsed from ESP-IDF `soc_reset_reason_t` in `reset_reasons.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` (VDD voltage
    /// not stable, resets the chip); that is not distinguishable in a Rust enum.
    ChipPowerOn   = 0x01,
    /// Software resets the digital core (hp system) by LP_AON_HPSYS_SW_RESET
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core (hp system)
    CoreDeepSleep = 0x05,
    /// Main watch dog 0 resets digital core (hp system)
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core (hp system)
    CoreMwdt1     = 0x08,
    /// RTC watch dog resets digital core (hp system)
    CoreRtcWdt    = 0x09,
    /// Main watch dog 0 resets CPU 0
    Cpu0Mwdt0     = 0x0B,
    /// Software resets CPU 0 by LP_AON_CPU_CORE0_SW_RESET
    Cpu0Sw        = 0x0C,
    /// RTC watch dog resets CPU 0
    Cpu0RtcWdt    = 0x0D,
    /// VDD voltage is not stable and resets the digital core
    SysBrownOut   = 0x0F,
    /// RTC watch dog resets digital core and rtc module
    SysRtcWdt     = 0x10,
    /// Main watch dog 1 resets CPU 0
    Cpu0Mwdt1     = 0x11,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt   = 0x12,
    /// eFuse CRC error resets the digital core (hp system)
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core (hp system)
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core (hp system)
    CoreUsbJtag   = 0x16,
    /// JTAG resets CPU 0
    Cpu0Jtag      = 0x18,
    /// RTC power glitch resets system
    RtcBrownOut   = 0x19,
    /// CPU lockup resets
    CpuLockup     = 0x1A,
}

use strum::FromRepr;

use crate::soc::clocks::ClockConfig;

pub(crate) fn init(_config: &ClockConfig) {}

/// SOC Reset Reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` or
    /// `ChipSuperWdt`, however that is not really compatible with Rust-style
    /// enums.
    ChipPowerOn      = 0x01,
    /// Software resets the digital core
    CoreSw           = 0x03,
    /// Deep sleep reset the digital core (also: PMU HP power down core reset)
    CoreDeepSleep    = 0x05,
    /// PMU HP CPU power down reset
    CpuPmuPwrDown    = 0x06,
    /// Main watch dog 0 resets digital core
    CoreMwdt0        = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1        = 0x08,
    /// RWDT core reset
    CoreRwdt         = 0x09,
    /// MWDT HP CPU reset
    CpuMwdt          = 0x0B,
    /// Software resets HP CPU
    CpuSw            = 0x0C,
    /// RWDT resets digital core
    CpuRwdt          = 0x0D,
    /// VDD voltage is not stable and resets the digital core
    SysBrownOut      = 0x0F,
    /// RWDT system reset
    SysRwdt          = 0x10,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt      = 0x12,
    /// Glitch on power resets the digital core and rtc module
    CorePwrGlitch    = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc     = 0x14,
    /// USB Serial/JTAG controller's JTAG resets the digital core
    CoreUsbJtag      = 0x16,
    /// USB Serial/JTAG controller's UART resets the digital core
    CoreUsbUart      = 0x17,
    /// JTAG resets the CPU
    CpuJtag          = 0x18,
    /// CPU lockup (exception inside exception handler)
    CpuLockup        = 0x1A,
}

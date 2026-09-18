use strum::FromRepr;

use crate::soc::{clocks::ClockConfig, regi2c};

pub(crate) fn init(_config: &ClockConfig) {
    calibrate_ocode();
}

/// Trims the RTC bandgap to the code measured for this die in the factory.
///
/// The SAR ADC's reference derives from that bandgap, so an untrimmed one reads as a gain error
/// against ESP-IDF.
///
/// ESP-IDF only does this coming out of a power-on reset; on any other reset the RTC domain still
/// holds the trim. Blocks other than version 1 carry no measured code, and are calibrated by a
/// software sweep that is not implemented here.
///
/// See `set_ocode_by_efuse` in
/// <https://github.com/espressif/esp-idf/blob/v6.1/components/esp_hw_support/port/esp32s3/rtc_init.c>
fn calibrate_ocode() {
    if crate::system::reset_reason() != Some(SocResetReason::ChipPowerOn)
        || crate::efuse::read_field_le::<u8>(crate::efuse::BLK_VERSION_MAJOR) != 1
    {
        return;
    }

    regi2c::I2C_ULP_EXT_CODE.write_reg(crate::efuse::read_field_le::<u8>(crate::efuse::OCODE));
    regi2c::I2C_ULP_IR_FORCE_CODE.write_field(1);
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

/// SOC Reset Reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Powers on reset.
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` or
    /// `ChipSuperWdt`, however that is not really compatible with Rust-style
    /// enums.
    ChipPowerOn   = 0x01,
    /// Software resets the digital core by RTC_CNTL_SW_SYS_RST
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core
    CoreDeepSleep = 0x05,
    /// Main watch dog 0 resets digital core
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1     = 0x08,
    /// RTC watch dog resets digital core
    CoreRtcWdt    = 0x09,
    /// Main watch dog 0 resets CPU
    ///
    /// In ESP-IDF there are `Cpu0Mwdt0` and `Cpu1Mwdt0`, however they have the
    /// same values.
    CpuMwdt0      = 0x0B,
    /// Software resets CPU by RTC_CNTL_SW_(PRO|APP)CPU_RST
    ///
    /// In ESP-IDF there are `Cpu0Sw` and `Cpu1Sw`, however they have the same
    /// values.
    CpuSw         = 0x0C,
    /// RTC watch dog resets CPU
    ///
    /// In ESP-IDF there are `Cpu0RtcWdt` and `Cpu1RtcWdt`, however they have
    /// the same values.
    CpuRtcWdt     = 0x0D,
    /// VDD voltage is not stable and resets the digital core.
    SysBrownOut   = 0x0F,
    /// RTC watch dog resets digital core and rtc module
    SysRtcWdt     = 0x10,
    /// Main watch dog 1 resets CPU
    ///
    /// In ESP-IDF there are `Cpu0Mwdt1` and `Cpu1Mwdt1`, however they have the
    /// same values.
    CpuMwdt1      = 0x11,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt   = 0x12,
    /// Glitch on clock resets the digital core and rtc module
    SysClkGlitch  = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// Glitch on power resets the digital core
    CorePwrGlitch = 0x17,
}

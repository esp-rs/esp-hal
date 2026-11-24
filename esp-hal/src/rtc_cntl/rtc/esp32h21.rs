use strum::FromRepr;

use crate::{
    peripherals::{LP_CLKRST, PMU},
    soc::{clocks::ClockConfig, regi2c},
};

// IDF sourced values, see `soc/rtc.h` and `pmu_param.h`.
const CK8M_DFREQ: u16 = 860;
const SCK_DCAP: u8 = 85;
const HP_CALI_DRVB: u32 = 8;
const LP_CALI_DBIAS: u8 = 0;
const DCDC_DREG: u8 = 18;

/// Mirrors the analog part of esp-idf's `rtc_clk_init()`.
pub(crate) fn init(_config: &ClockConfig) {
    // Tune the RC oscillators. On this chip, RC32K_DFREQ tunes RC_SLOW.
    LP_CLKRST::regs()
        .fosc_cntl()
        .modify(|_, w| unsafe { w.fosc_dfreq().bits(CK8M_DFREQ) });
    LP_CLKRST::regs()
        .rc32k_cntl()
        .modify(|_, w| unsafe { w.rc32k_dfreq().bits(SCK_DCAP as _) });

    // Switch the DCDC to CCM mode.
    PMU::regs()
        .dcm_ctrl()
        .modify(|_, w| w.dcdc_ccm_sw_en().set_bit());
    PMU::regs()
        .hp_active_bias()
        .modify(|_, w| w.hp_active_dcdc_ccm_enb().clear_bit());

    regi2c::I2C_DCDC_CCM_DREG0.write_field(DCDC_DREG);
    regi2c::I2C_DCDC_CCM_PCUR_LIMIT0.write_field(4);
    regi2c::I2C_DCDC_VCM_DREG0.write_field(DCDC_DREG);
    regi2c::I2C_DCDC_VCM_PCUR_LIMIT0.write_field(2);
    regi2c::I2C_DCDC_XPD_TRX.write_field(0);

    // Bit 25 of the PMU date register controls the RF PLL.
    PMU::regs()
        .date()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 25)) });

    PMU::regs()
        .hp_active_hp_regulator0()
        .modify(|_, w| w.hp_active_hp_regulator_xpd().set_bit());
    PMU::regs()
        .hp_active_hp_regulator1()
        .modify(|_, w| unsafe { w.hp_active_hp_regulator_drv_b().bits(HP_CALI_DRVB) });
    PMU::regs()
        .hp_sleep_lp_regulator0()
        .modify(|_, w| unsafe { w.hp_sleep_lp_regulator_dbias().bits(LP_CALI_DBIAS) });
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
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut`, however
    /// that is not really compatible with Rust-style enums.
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
    /// Main watch dog 0 resets CPU 0
    Cpu0Mwdt0     = 0x0B,
    /// Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST
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
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// Glitch on power resets the digital core
    CorePwrGlitch = 0x17,
    /// JTAG resets the CPU 0
    Cpu0Jtag      = 0x18,
}

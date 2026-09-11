use strum::FromRepr;

use crate::{
    peripherals::{LP_CLKRST, PMU},
    soc::{clocks::ClockConfig, regi2c},
};

// IDF sourced values.
const CK8M_DFREQ: u16 = 100;
const SCK_DCAP: u16 = 28;
const HP_CALI_DRVB: u32 = 6;
const LP_CALI_DBIAS: u8 = 0;

/// Clears the force bits of the power domains, same as ESP32-H2.
fn pmu_power_domain_force_default() {
    let pmu = PMU::regs();
    unsafe {
        pmu.power_pd_top_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpaon_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpcpu_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpperi_reserve().write(|w| w.bits(0));
        pmu.power_pd_hpwifi_cntl().write(|w| w.bits(0));
        pmu.power_pd_lpperi_cntl().write(|w| w.bits(0));
    }
}

pub(crate) fn init(_config: &ClockConfig) {
    // This is the part of esp-idf's `rtc_clk_init` that has to run before the CPU can be switched
    // to the BBPLL. The sleep-related defaults of `pmu_init` are left to the sleep driver.

    // Tuning of the internal oscillators. Unlike on the other chips, RC_SLOW is tuned through the
    // RC32K register (see the comment in esp-idf's rtc_clk_init.c).
    LP_CLKRST::regs()
        .fosc_cntl()
        .modify(|_, w| unsafe { w.fosc_dfreq().bits(CK8M_DFREQ) });
    LP_CLKRST::regs()
        .rc32k_cntl()
        .modify(|_, w| unsafe { w.rc32k_dfreq().bits(SCK_DCAP) });

    // Switch the DC-DC converter to CCM mode and apply its defaults.
    PMU::regs()
        .dcm_ctrl()
        .modify(|_, w| w.dcdc_ccm_sw_en().set_bit());
    PMU::regs()
        .hp_active_bias()
        .modify(|_, w| w.hp_active_dcdc_ccm_enb().clear_bit());

    regi2c::I2C_DCDC_CCM_DREG0.write_field(24);
    regi2c::I2C_DCDC_CCM_PCUR_LIMIT0.write_field(4);
    regi2c::I2C_DCDC_VCM_DREG0.write_field(24);
    regi2c::I2C_DCDC_VCM_PCUR_LIMIT0.write_field(2);
    regi2c::I2C_DCDC_XPD_TRX.write_field(0);

    // Regulator settings for the active modes.
    PMU::regs()
        .hp_active_hp_regulator0()
        .modify(|_, w| w.hp_active_hp_regulator_xpd().set_bit());
    PMU::regs()
        .hp_active_hp_regulator1()
        .modify(|_, w| unsafe { w.hp_active_hp_regulator_drv_b().bits(HP_CALI_DRVB) });
    // The LP active mode registers are the ones named after HP sleep.
    PMU::regs()
        .hp_sleep_lp_regulator0()
        .modify(|_, w| unsafe { w.hp_sleep_lp_regulator_dbias().bits(LP_CALI_DBIAS) });

    // pmu_init()
    pmu_power_domain_force_default();
}

/// SOC Reset Reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Powers on reset.
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut`
    ChipPowerOn   = 0x01,
    /// Software resets the digital core by LP_AON_HPSYS_SW_RESET
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
    /// Glitch on power resets the digital core and rtc module
    CorePwrGlitch = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB UART resets the digital core
    CoreUsbUart   = 0x15,
    /// USB JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// JTAG resets CPU 0
    Cpu0JtagCpu   = 0x18,
    /// CPU lockup reset
    CpuLockup     = 0x1A,
}

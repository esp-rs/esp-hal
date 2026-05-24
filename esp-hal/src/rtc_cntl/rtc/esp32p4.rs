//! RTC control for ESP32-P4X (chip revision v3.x / eco5).
//!
//! PMU power domain init and WDT disable for bare-metal boot.
//!
//! Register names validated against esp32p4 PAC (eco5) and:
//!   - TRM v0.5 Ch 16 (Low-Power Management)
//!   - TRM v0.5 Ch 19 (Watchdog Timers)
//!
//! eco4 vs eco5 PMU difference:
//!   eco4: power_pd_hpaon_cntl, power_pd_hpcpu_cntl, power_pd_hpwifi_cntl
//!   eco5: power_pd_cnnt_cntl, power_pd_lpperi_cntl (hpaon/hpcpu/hpwifi removed)
//!   Both: power_pd_top_cntl, power_pd_hpmem_cntl

// TODO: REMOVE me when validate well
// reference florianL21's original eco4 PMU code

use strum::FromRepr;

use crate::{
    peripherals::{HP_SYS_CLKRST, LP_AON_CLKRST, LP_WDT, PMU, TIMG0, TIMG1},
    soc::regi2c,
};

/// SOC Reset Reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
#[repr(usize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SocResetReason {
    /// Power on reset
    ChipPowerOn   = 0x01,
    /// Software resets the digital core
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core
    CoreDeepSleep = 0x05,
    /// SDIO Core reset
    CoreSDIO      = 0x06,
    /// Main watch dog 0 resets digital core
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1     = 0x08,
    /// RTC watch dog resets digital core
    CoreRtcWdt    = 0x09,
    /// Main watch dog 0 resets CPU 0
    Cpu0Mwdt0     = 0x0B,
    /// Software resets CPU 0
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
    /// JTAG resets CPU
    Cpu0JtagCpu   = 0x18,
}

/// Clear all force flags on PMU power domains to allow normal power management.
///
/// eco5 power domains (verified against PAC):
///   - TOP: power_pd_top_cntl (system top-level)
///   - CNNT: power_pd_cnnt_cntl (connectivity, eco5 new -- replaces hpaon/hpcpu/hpwifi)
///   - HPMEM: power_pd_hpmem_cntl (HP memory)
///   - LPPERI: power_pd_lpperi_cntl (LP peripherals, eco5 new)
fn pmu_power_domain_force_default() {
    let pmu = PMU::regs();

    // PMU_HP_PD_TOP
    pmu.power_pd_top_cntl().modify(|_, w| {
        w.force_top_reset().bit(false);
        w.force_top_iso().bit(false);
        w.force_top_pu().bit(false);
        w.force_top_no_reset().bit(false);
        w.force_top_no_iso().bit(false);
        w.force_top_pd().bit(false)
    });

    // PMU_HP_PD_CNNT (eco5: replaces hpaon + hpcpu + hpwifi from eco4)
    pmu.power_pd_cnnt_cntl().modify(|_, w| {
        w.force_cnnt_reset().bit(false);
        w.force_cnnt_iso().bit(false);
        w.force_cnnt_pu().bit(false);
        w.force_cnnt_no_reset().bit(false);
        w.force_cnnt_no_iso().bit(false);
        w.force_cnnt_pd().bit(false)
    });

    // PMU_HP_PD_HPMEM
    pmu.power_pd_hpmem_cntl().modify(|_, w| {
        w.force_hp_mem_reset().bit(false);
        w.force_hp_mem_iso().bit(false);
        w.force_hp_mem_pu().bit(false);
        w.force_hp_mem_no_reset().bit(false);
        w.force_hp_mem_no_iso().bit(false);
        w.force_hp_mem_pd().bit(false)
    });

    // PMU_LP_PD_LPPERI (eco5 new)
    pmu.power_pd_lpperi_cntl().modify(|_, w| {
        w.force_lp_peri_reset().bit(false);
        w.force_lp_peri_iso().bit(false);
        w.force_lp_peri_pu().bit(false);
        w.force_lp_peri_no_reset().bit(false);
        w.force_lp_peri_no_iso().bit(false);
        w.force_lp_peri_pd().bit(false)
    });
}

/// Minimal init for bare-metal boot: disable all watchdogs and set power domains.
///
/// This is the minimum needed for the chip to not reset itself during startup.
/// Full clock configuration (PLL, CPU freq) is handled separately.
///
/// Ref: TRM v0.5 Ch 19 (WDT), Ch 16 (Power Management)
pub(crate) fn init() {
    // 1. Clear all PMU power domain force flags
    pmu_power_domain_force_default();

    // 2. Disable TIMG0 watchdog TIMG WDT write protect key: 0x50D8_3AA1
    let tg0 = TIMG0::regs();
    tg0.wdtwprotect().write(|w| unsafe { w.bits(0x50D8_3AA1) });
    tg0.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
    tg0.wdtwprotect().write(|w| unsafe { w.bits(0) });

    // 3. Disable TIMG1 watchdog
    let tg1 = TIMG1::regs();
    tg1.wdtwprotect().write(|w| unsafe { w.bits(0x50D8_3AA1) });
    tg1.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
    tg1.wdtwprotect().write(|w| unsafe { w.bits(0) });

    // 4. Disable LP_WDT (Low-Power Watchdog) P4 PAC: config0() (not wdtconfig0()), wprotect() (not
    //    wdtwprotect())
    let lp_wdt = LP_WDT::regs();
    lp_wdt.wprotect().write(|w| unsafe { w.bits(0x50D8_3AA1) });
    lp_wdt.config0().modify(|_, w| unsafe { w.bits(0) });
    lp_wdt.wprotect().write(|w| unsafe { w.bits(0) });

    // 5. Disable SWD (Super Watchdog) by enabling auto-feed LP_WDT SWD write protect key:
    //    0x50D8_3AA1 (same key)
    lp_wdt
        .swd_wprotect()
        .write(|w| unsafe { w.bits(0x50D8_3AA1) });
    lp_wdt
        .swd_config()
        .modify(|_, w| w.swd_auto_feed_en().set_bit());
    lp_wdt.swd_wprotect().write(|w| unsafe { w.bits(0) });

    // 6. Clear DCDC switch force flags PMU_POWER_DCDC_SWITCH_REG (offset 0x10c)
    PMU::regs().power_dcdc_switch().modify(|_, w| {
        w.force_dcdc_switch_pu().bit(false);
        w.force_dcdc_switch_pd().bit(false)
    });

    // 7. Enable CPLL (400 MHz) and SPLL (480 MHz)
    cpll_configure(400);
    spll_configure(480);

    // 8. Set CPU divider to 1 (400 MHz CPU), MEM divider 2, APB divider 2
    // Set CPU divider: cpu_clk_div_num = divider - 1 = 0
    // Ref: HP_SYS_CLKRST.root_clk_ctrl0.reg_cpu_clk_div_num
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .modify(|_, w| unsafe {
            w.cpu_clk_div_num().bits(0); // CPU: /1 = 400 MHz
            w.cpu_clk_div_numerator().bits(0);
            w.cpu_clk_div_denominator().bits(0)
        });
    // Trigger clock divider update
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .modify(|_, w| w.soc_clk_div_update().set_bit());
    while HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .read()
        .soc_clk_div_update()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }

    // 9. Switch CPU clock source from XTAL to CPLL LP_AON_CLKRST.hp_clk_ctrl.hp_root_clk_src_sel:
    //    0=XTAL, 1=CPLL, 2=RC_FAST
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| unsafe { w.lp_aonclkrst_hp_root_clk_src_sel().bits(1) }); // 1 = CPLL
}

/// Configure CPLL for the given frequency (360 or 400 MHz).
///
/// eco5 (v3.x) uses different I2C register values than eco4 (v1.x).
/// Ref: TRM v0.5 Ch 12 -- CPLL configuration
fn cpll_configure(freq_mhz: u32) {
    // 1. Enable CPLL power PMU.imm_hp_ck_power: tie_high_xpd_pll, tie_high_xpd_pll_i2c Note: PAC
    //    uses "pll" not "cpll" (eco4 PAC, single PLL)
    // PAC: tie_high_xpd_pll is 4-bit field (one bit per PLL: CPLL/SPLL/MPLL/PLLA).
    // Set all bits to enable all PLLs. Same for pll_i2c.
    PMU::regs().imm_hp_ck_power().write(|w| unsafe {
        w.tie_high_xpd_pll().bits(0xF);
        w.tie_high_xpd_pll_i2c().bits(0xF)
    });
    // Enable global CPLL ICG (clock gating)
    // Ref: PMU.imm_hp_ck_power.tie_high_global_cpll_icg
    // Note: this field may not exist in eco4 PAC; use direct bit write if needed
    // For now, the PLL is enabled via xpd_pll above.

    // 2. Configure CPLL via I2C analog registers
    // eco5 values (v3.x): div7_0 = 10 (400MHz), 9 (360MHz) with 40MHz XTAL
    let div7_0: u8 = match freq_mhz {
        400 => 10, // eco5: 400 MHz
        360 => 9,  // eco5: 360 MHz
        _ => 10,   // default to 400 MHz
    };

    // OC_REF_DIV + DCHGP: (oc_enb_fcal << 7) | (dchgp << 4) | div_ref = 0x50
    let lref: u8 = 0x50; // dchgp=5, div_ref=0, oc_enb_fcal=0

    // OC_DCUR: (dlref_sel << 6) | (dhref_sel << 4) | dcur = 0x73
    let dcur: u8 = 0x73; // dlref_sel=1, dhref_sel=3, dcur=3

    regi2c::I2C_CPLL_OC_REF_DIV.write_reg(lref);
    regi2c::I2C_CPLL_OC_DIV_7_0.write_reg(div7_0);
    regi2c::I2C_CPLL_OC_DCUR.write_reg(dcur);

    // 3. Run CPLL calibration HP_SYS_CLKRST.ana_pll_ctrl0.cpu_pll_cal_stop
    // Start calibration: set cpu_pll_cal_stop = 0
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().clear_bit());

    // Wait for calibration to complete
    while !HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .cpu_pll_cal_end()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }

    // Stop calibration: set cpu_pll_cal_stop = 1
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().set_bit());

    // Small delay for PLL to stabilize
    crate::rom::ets_delay_us(10);
}

/// Configure SPLL (System PLL) for the given frequency (480 MHz typical).
///
/// SPLL provides peripheral clocks: PLL_F240M/160M/120M/80M/20M.
/// Ref: TRM v0.5 Ch 12 -- SPLL configuration
fn spll_configure(freq_mhz: u32) {
    // PLL power already enabled in cpll_configure (PMU xpd_pll bits(0xF) enables all PLLs)

    // Configure SPLL via I2C analog registers
    // eco5 values: div7_0 = (freq_mhz / 40) - 1, same formula as CPLL
    let div7_0: u8 = match freq_mhz {
        480 => 11, // 480 MHz: (480/40) - 1 = 11
        240 => 5,  // 240 MHz: (240/40) - 1 = 5
        _ => 11,   // default to 480 MHz
    };

    // Same OC_REF_DIV and OC_DCUR values as CPLL
    let lref: u8 = 0x50; // dchgp=5, div_ref=0, oc_enb_fcal=0
    let dcur: u8 = 0x73; // dlref_sel=1, dhref_sel=3, dcur=3

    regi2c::I2C_SPLL_OC_REF_DIV.write_reg(lref);
    regi2c::I2C_SPLL_OC_DIV_7_0.write_reg(div7_0);
    regi2c::I2C_SPLL_OC_DCUR.write_reg(dcur);

    // Run SPLL calibration
    //      HP_SYS_CLKRST.ana_pll_ctrl0.sys_pll_cal_stop
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.sys_pll_cal_stop().clear_bit());

    while !HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .sys_pll_cal_end()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }

    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.sys_pll_cal_stop().set_bit());

    crate::rom::ets_delay_us(10);
}

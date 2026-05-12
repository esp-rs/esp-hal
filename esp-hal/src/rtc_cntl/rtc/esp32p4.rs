//! RTC control for ESP32-P4X (eco5 PMU, chip rev v3.x).

use strum::FromRepr;

use crate::peripherals::{LP_WDT, PMU, TIMG0, TIMG1};

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

// Clear all force_* overrides on the eco5 PMU power domains so the FSM
// can manage them normally.
fn pmu_power_domain_force_default() {
    let pmu = PMU::regs();

    pmu.power_pd_top_cntl().modify(|_, w| {
        w.force_top_reset().bit(false);
        w.force_top_iso().bit(false);
        w.force_top_pu().bit(false);
        w.force_top_no_reset().bit(false);
        w.force_top_no_iso().bit(false);
        w.force_top_pd().bit(false)
    });

    pmu.power_pd_cnnt_cntl().modify(|_, w| {
        w.force_cnnt_reset().bit(false);
        w.force_cnnt_iso().bit(false);
        w.force_cnnt_pu().bit(false);
        w.force_cnnt_no_reset().bit(false);
        w.force_cnnt_no_iso().bit(false);
        w.force_cnnt_pd().bit(false)
    });

    pmu.power_pd_hpmem_cntl().modify(|_, w| {
        w.force_hp_mem_reset().bit(false);
        w.force_hp_mem_iso().bit(false);
        w.force_hp_mem_pu().bit(false);
        w.force_hp_mem_no_reset().bit(false);
        w.force_hp_mem_no_iso().bit(false);
        w.force_hp_mem_pd().bit(false)
    });

    pmu.power_pd_lpperi_cntl().modify(|_, w| {
        w.force_lp_peri_reset().bit(false);
        w.force_lp_peri_iso().bit(false);
        w.force_lp_peri_pu().bit(false);
        w.force_lp_peri_no_reset().bit(false);
        w.force_lp_peri_no_iso().bit(false);
        w.force_lp_peri_pd().bit(false)
    });
}

// WDT write-protect unlock key.
const WDT_WKEY: u32 = 0x50D8_3AA1;

pub(crate) fn init() {
    pmu_power_domain_force_default();

    let tg0 = TIMG0::regs();
    tg0.wdtwprotect().write(|w| unsafe { w.bits(WDT_WKEY) });
    tg0.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
    tg0.wdtwprotect().write(|w| unsafe { w.bits(0) });

    let tg1 = TIMG1::regs();
    tg1.wdtwprotect().write(|w| unsafe { w.bits(WDT_WKEY) });
    tg1.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
    tg1.wdtwprotect().write(|w| unsafe { w.bits(0) });

    let lp_wdt = LP_WDT::regs();
    lp_wdt.wprotect().write(|w| unsafe { w.bits(WDT_WKEY) });
    lp_wdt.config0().modify(|_, w| unsafe { w.bits(0) });
    lp_wdt.wprotect().write(|w| unsafe { w.bits(0) });

    lp_wdt.swd_wprotect().write(|w| unsafe { w.bits(WDT_WKEY) });
    lp_wdt
        .swd_config()
        .modify(|_, w| w.swd_auto_feed_en().set_bit());
    lp_wdt.swd_wprotect().write(|w| unsafe { w.bits(0) });

    PMU::regs().power_dcdc_switch().modify(|_, w| {
        w.force_dcdc_switch_pu().bit(false);
        w.force_dcdc_switch_pd().bit(false)
    });

    cpll_configure(400);
    spll_configure(480);

    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    clkrst.root_clk_ctrl0().modify(|_, w| unsafe {
        w.cpu_clk_div_num().bits(0); // CPU: /1 = 400 MHz
        w.cpu_clk_div_numerator().bits(0);
        w.cpu_clk_div_denominator().bits(0)
    });
    clkrst
        .root_clk_ctrl0()
        .modify(|_, w| w.soc_clk_div_update().set_bit());
    while clkrst
        .root_clk_ctrl0()
        .read()
        .soc_clk_div_update()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }

    // hp_root_clk_src_sel: 0=XTAL, 1=CPLL, 2=RC_FAST.
    crate::peripherals::LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| unsafe { w.lp_aonclkrst_hp_root_clk_src_sel().bits(1) });
}

// Configure CPLL (360 or 400 MHz) via REGI2C analog trims, then calibrate.
fn cpll_configure(freq_mhz: u32) {
    use crate::soc::regi2c;

    const I2C_CPLL: u8 = 0x67;
    const I2C_CPLL_OC_REF_DIV: u8 = 2;
    const I2C_CPLL_OC_DIV_7_0: u8 = 3;
    const I2C_CPLL_OC_DCUR: u8 = 6;

    // tie_high_xpd_pll/pll_i2c are 4-bit (CPLL/SPLL/MPLL/PLLA).
    PMU::regs().imm_hp_ck_power().write(|w| unsafe {
        w.tie_high_xpd_pll().bits(0xF);
        w.tie_high_xpd_pll_i2c().bits(0xF)
    });

    // eco5 with 40 MHz XTAL.
    let div7_0: u8 = match freq_mhz {
        400 => 10,
        360 => 9,
        _ => 10,
    };
    let lref: u8 = 0x50; // dchgp=5, div_ref=0, oc_enb_fcal=0
    let dcur: u8 = 0x73; // dlref_sel=1, dhref_sel=3, dcur=3

    regi2c::regi2c_write(I2C_CPLL, 0, I2C_CPLL_OC_REF_DIV, lref);
    regi2c::regi2c_write(I2C_CPLL, 0, I2C_CPLL_OC_DIV_7_0, div7_0);
    regi2c::regi2c_write(I2C_CPLL, 0, I2C_CPLL_OC_DCUR, dcur);

    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    clkrst
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().clear_bit());
    while !clkrst.ana_pll_ctrl0().read().cpu_pll_cal_end().bit_is_set() {
        core::hint::spin_loop();
    }
    clkrst
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().set_bit());

    crate::rom::ets_delay_us(10);
}

// Configure SPLL (480 or 240 MHz). SPLL feeds PLL_F240M/160M/120M/80M/20M.
fn spll_configure(freq_mhz: u32) {
    use crate::soc::regi2c;

    // PLL power was enabled in cpll_configure (xpd_pll covers all PLLs).
    let div7_0: u8 = match freq_mhz {
        480 => 11,
        240 => 5,
        _ => 11,
    };
    let lref: u8 = 0x50;
    let dcur: u8 = 0x73;

    regi2c::I2C_SPLL_OC_REF_DIV.write_reg(lref);
    regi2c::I2C_SPLL_OC_DIV_7_0.write_reg(div7_0);
    regi2c::I2C_SPLL_OC_DCUR.write_reg(dcur);

    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    clkrst
        .ana_pll_ctrl0()
        .modify(|_, w| w.sys_pll_cal_stop().clear_bit());
    while !clkrst.ana_pll_ctrl0().read().sys_pll_cal_end().bit_is_set() {
        core::hint::spin_loop();
    }
    clkrst
        .ana_pll_ctrl0()
        .modify(|_, w| w.sys_pll_cal_stop().set_bit());

    crate::rom::ets_delay_us(10);
}

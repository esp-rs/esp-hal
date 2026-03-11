use strum::FromRepr;

use crate::{peripherals::PMU, soc::regi2c};

pub(crate) fn init() {
    // * No peripheral reg i2c power up required on the target */
    regi2c::I2C_PMU_EN_I2C_RTC_DREG.write_field(0);
    regi2c::I2C_PMU_EN_I2C_DIG_DREG.write_field(0);
    regi2c::I2C_PMU_EN_I2C_RTC_DREG_SLP.write_field(0);
    regi2c::I2C_PMU_EN_I2C_DIG_DREG_SLP.write_field(0);
    regi2c::I2C_PMU_OR_XPD_RTC_REG.write_field(0);
    regi2c::I2C_PMU_OR_XPD_DIG_REG.write_field(0);
    regi2c::I2C_PMU_OR_XPD_TRX.write_field(0);
    regi2c::I2C_BIAS_DREG_0P8.write_field(8);

    let pmu = PMU::regs();
    unsafe {
        pmu.power_pd_top_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpaon_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpcpu_cntl().write(|w| w.bits(0));
        pmu.power_pd_hpperi_reserve().write(|w| w.bits(0));
        pmu.power_pd_hpwifi_cntl().write(|w| w.bits(0));
        pmu.power_pd_lpperi_cntl().write(|w| w.bits(0));

        pmu.hp_active_hp_regulator0()
            .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(25));
        pmu.hp_sleep_lp_regulator0()
            .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(26));

        pmu.hp_sleep_dig_power().modify(|_, w| {
            w.hp_sleep_vdd_spi_pd_en()
                .set_bit()
                .hp_sleep_pd_hp_wifi_pd_en()
                .set_bit()
                .hp_sleep_pd_hp_cpu_pd_en()
                .set_bit()
                .hp_sleep_pd_top_pd_en()
                .set_bit()
        });

        pmu.hp_active_hp_ck_power().modify(|_, w| {
            w.hp_active_xpd_bbpll()
                .set_bit()
                .hp_active_xpd_bb_i2c()
                .set_bit()
                .hp_active_xpd_bbpll_i2c()
                .set_bit()
        });

        pmu.hp_active_sysclk().modify(|_, w| {
            w.hp_active_icg_sys_clock_en()
                .set_bit()
                .hp_active_sys_clk_slp_sel()
                .clear_bit()
                .hp_active_icg_slp_sel()
                .clear_bit()
        });
        pmu.hp_sleep_sysclk().modify(|_, w| {
            w.hp_sleep_icg_sys_clock_en()
                .clear_bit()
                .hp_sleep_sys_clk_slp_sel()
                .set_bit()
                .hp_sleep_icg_slp_sel()
                .set_bit()
        });

        pmu.slp_wakeup_cntl5()
            .modify(|_, w| w.lp_ana_wait_target().bits(15));
        pmu.slp_wakeup_cntl7()
            .modify(|_, w| w.ana_wait_target().bits(1700));
    }
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

bitfield::bitfield! {
    /// Representation of `PMU_HP_{ACTIVE,SLEEP}_DIG_POWER_REG` registers.
    #[derive(Clone, Copy, Default)]
    pub struct HpDigPower(u32);

    pub bool, vdd_spi_pd_en, set_vdd_spi_pd_en: 21;
    pub bool, mem_dslp     , set_mem_dslp     : 22;
    pub bool, modem_pd_en  , set_modem_pd_en  : 27;
    pub bool, cpu_pd_en    , set_cpu_pd_en    : 29;
    pub bool, top_pd_en    , set_top_pd_en    : 31;
}

bitfield::bitfield! {
    /// Representation of `PMU_HP_{ACTIVE,SLEEP}_HP_CK_POWER_REG` registers.
    #[derive(Clone, Copy, Default)]
    pub struct HpClkPower(u32);

    pub bool, xpd_bbpll    , set_xpd_bbpll    : 30;
}

bitfield::bitfield! {
    /// Representation of `PMU_{HP_ACTIVE,HP_SLEEP,LP_SLEEP}_XTAL_REG` register.
    #[derive(Clone, Copy, Default)]
    pub struct XtalPower(u32);

    pub bool, xpd_xtal     , set_xpd_xtal     : 31;
}

/// Combined HP system power settings.
#[derive(Clone, Copy, Default)]
pub struct HpSysPower {
    pub dig_power: HpDigPower,
    pub clk: HpClkPower,
    pub xtal: XtalPower,
}

bitfield::bitfield! {
    /// Representation of `PMU_{HP,LP}_SLEEP_LP_DIG_POWER_REG`.
    #[derive(Clone, Copy, Default)]
    pub struct LpDigPower(u32);

    pub bool, bod_source_sel, set_bod_source_sel : 27;
    pub u32, vddbat_mode, set_vddbat_mode : 29, 28;
    pub u32, mem_dslp  , set_mem_dslp  : 30;

}

bitfield::bitfield! {
    /// Representation of `PMU_{HP,LP}_SLEEP_LP_CK_POWER_REG`.
    #[derive(Clone, Copy, Default)]
    pub struct LpClkPower(u32);

    pub u32, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub u32, xpd_fosc   , set_xpd_fosc   : 30;
}

/// Combined LP system power settings.
#[derive(Clone, Copy, Default)]
pub struct LpSysPower {
    pub dig_power: LpDigPower,
    pub clk_power: LpClkPower,
    pub xtal: XtalPower,
}

bitfield::bitfield! {
    /// Representation of `PMU_HP_{ACTIVE,SLEEP}_HP_SYS_CNTL_REG` register.
    #[derive(Clone, Copy, Default)]
    pub struct HpSysCntlReg(u32);

    pub bool, uart_wakeup_en , set_uart_wakeup_en : 24;
    pub bool, lp_pad_hold_all, set_lp_pad_hold_all: 25;
    pub bool, hp_pad_hold_all, set_hp_pad_hold_all: 26;
    pub bool, dig_pad_slp_sel, set_dig_pad_slp_sel: 27;
    pub bool, dig_pause_wdt  , set_dig_pause_wdt  : 28;
    pub bool, dig_cpu_stall  , set_dig_cpu_stall  : 29;
}

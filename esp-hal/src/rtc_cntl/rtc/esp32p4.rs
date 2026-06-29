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
    soc::{
        clocks::{self, ClockTree, CpuRootClkConfig},
        regi2c,
    },
};

// `dig_sysclk_sel` codes (SOC_CPU_CLK_SRC_*).
const HP_SYSCLK_XTAL: u8 = 0;

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

// ---------------------------------------------------------------------------
// PMU register data structures.
//
// These mirror the `pmu_*_t` structs in esp-idf. The bit positions match the
// ESP32-P4 PMU PAC register layouts so that the power configs can be written as
// raw `u32` values, while the analog/syscntl configs are written field-by-field
// from the sleep driver.
// ---------------------------------------------------------------------------

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.0 (hp_*_dig_power)
    pub struct HpDigPower(u32);

    pub bool, dcdc_switch_pd_en, set_dcdc_switch_pd_en: 21;
    pub bool, mem_dslp          , set_mem_dslp          : 22;
    pub bool, mem_pd_en         , set_mem_pd_en         : 23;
    pub bool, cpu_pd_en         , set_cpu_pd_en         : 29;
    pub bool, cnnt_pd_en        , set_cnnt_pd_en        : 30;
    pub bool, top_pd_en         , set_top_pd_en         : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.1 (hp_*_hp_ck_power)
    pub struct HpClkPower(u32);

    pub bool, i2c_iso_en   , set_i2c_iso_en   : 21;
    pub bool, i2c_retention, set_i2c_retention: 22;
    pub u8,   xpd_pll_i2c  , set_xpd_pll_i2c  : 26, 23;
    pub u8,   xpd_pll      , set_xpd_pll      : 30, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.2 (hp_*_xtal)
    pub struct HpXtalPower(u32);

    pub bool, xpd_xtal, set_xpd_xtal: 31;
}

#[derive(Clone, Copy, Default)]
// pmu_sleep_power_config_t.0
pub struct HpSysPower {
    pub dig_power: HpDigPower,
    pub clk: HpClkPower,
    pub xtal: HpXtalPower,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_sys_cntl_reg_t (hp_*_hp_sys_cntl)
    pub struct HpSysCntlReg(u32);

    pub bool, uart_wakeup_en , set_uart_wakeup_en : 24;
    pub bool, lp_pad_hold_all, set_lp_pad_hold_all: 25;
    pub bool, hp_pad_hold_all, set_hp_pad_hold_all: 26;
    pub bool, dig_pad_slp_sel, set_dig_pad_slp_sel: 27;
    pub bool, dig_pause_wdt  , set_dig_pause_wdt  : 28;
    pub bool, dig_cpu_stall  , set_dig_cpu_stall  : 29;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_sysclk_reg_t (hp_*_sysclk)
    pub struct HpSysclk(u32);

    pub bool, dig_sysclk_nodiv, set_dig_sysclk_nodiv: 26;
    pub bool, icg_sysclk_en   , set_icg_sysclk_en   : 27;
    pub bool, sysclk_slp_sel  , set_sysclk_slp_sel  : 28;
    pub bool, icg_slp_sel     , set_icg_slp_sel     : 29;
    pub u8,   dig_sysclk_sel  , set_dig_sysclk_sel  : 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.0 (hp_*_bias) -- note P4-only dcm fields.
    pub struct HpAnalogBias(u32);

    pub u8,   dcm_vset  , set_dcm_vset  : 22, 18;
    pub u8,   dcm_mode  , set_dcm_mode  : 24, 23;
    pub bool, xpd_bias  , set_xpd_bias  : 25;
    pub u8,   dbg_atten , set_dbg_atten : 29, 26;
    pub bool, pd_cur    , set_pd_cur    : 30;
    pub bool, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.1 (hp_*_hp_regulator0)
    pub struct HpAnalogRegulator0(u32);

    pub bool, slp_mem_xpd    , set_slp_mem_xpd    : 16;
    pub bool, slp_logic_xpd  , set_slp_logic_xpd  : 17;
    pub bool, xpd            , set_xpd            : 18;
    pub u8,   slp_mem_dbias  , set_slp_mem_dbias  : 22, 19;
    pub u8,   slp_logic_dbias, set_slp_logic_dbias: 26, 23;
    pub u8,   dbias          , set_dbias          : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.2 (hp_*_hp_regulator1)
    pub struct HpAnalogRegulator1(u32);

    pub u8, drv_b, set_drv_b: 31, 26;
}

#[derive(Clone, Copy, Default)]
// pmu_hp_analog_t
pub struct HpAnalog {
    pub bias: HpAnalogBias,
    pub regulator0: HpAnalogRegulator0,
    pub regulator1: HpAnalogRegulator1,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.0 (lp dig_power)
    pub struct LpDigPower(u32);

    pub bool, lp_pad_slp_sel, set_lp_pad_slp_sel: 26;
    pub bool, bod_source_sel, set_bod_source_sel: 27;
    pub u8,   vddbat_mode   , set_vddbat_mode   : 29, 28;
    pub bool, mem_dslp      , set_mem_dslp      : 30;
    pub bool, peri_pd_en    , set_peri_pd_en    : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.1 (lp clk_power)
    pub struct LpClkPower(u32);

    pub bool, xpd_lppll  , set_xpd_lppll  : 27;
    pub bool, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub bool, xpd_rc32k  , set_xpd_rc32k  : 29;
    pub bool, xpd_fosc   , set_xpd_fosc   : 30;
    pub bool, pd_osc     , set_pd_osc     : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.2 (lp xtal)
    pub struct LpXtalPower(u32);

    pub bool, xpd_xtal, set_xpd_xtal: 31;
}

#[derive(Clone, Copy, Default)]
// pmu_sleep_power_config_t.1
pub struct LpSysPower {
    pub dig_power: LpDigPower,
    pub clk_power: LpClkPower,
    pub xtal: LpXtalPower,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.0 (lp_*_bias)
    pub struct LpAnalogBias(u32);

    pub bool, xpd_bias  , set_xpd_bias  : 25;
    pub u8,   dbg_atten , set_dbg_atten : 29, 26;
    pub bool, pd_cur    , set_pd_cur    : 30;
    pub bool, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.1 (lp_*_lp_regulator0)
    pub struct LpAnalogRegulator0(u32);

    pub bool, slp_xpd  , set_slp_xpd  : 21;
    pub bool, xpd      , set_xpd      : 22;
    pub u8,   slp_dbias, set_slp_dbias: 26, 23;
    pub u8,   dbias    , set_dbias    : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.2 (lp_*_lp_regulator1)
    pub struct LpAnalogRegulator1(u32);

    pub u8, drv_b, set_drv_b: 31, 26;
}

#[derive(Clone, Copy, Default)]
// pmu_lp_analog_t
pub struct LpAnalog {
    pub bias: LpAnalogBias,
    pub regulator0: LpAnalogRegulator0,
    pub regulator1: LpAnalogRegulator1,
}

/// Configures the default per-mode HP system PMU registers (HP_ACTIVE and
/// HP_SLEEP). Mirrors `pmu_hp_system_init()` in esp-idf. The HP_MODEM mode is
/// skipped (P4 has no wireless modem).
fn pmu_hp_system_init() {
    let pmu = PMU::regs();

    // ---- HP_ACTIVE ----
    pmu.hp_active_dig_power().write(|w| unsafe { w.bits(0) });
    pmu.hp_active_hp_ck_power().write(|w| unsafe {
        w.hp_active_i2c_iso_en().bit(false);
        w.hp_active_i2c_retention().bit(false);
        w.hp_active_xpd_pll_i2c().bits(0xf);
        w.hp_active_xpd_pll().bits(0xf)
    });
    pmu.hp_active_xtal()
        .modify(|_, w| w.hp_active_xpd_xtal().bit(true));

    pmu.hp_active_icg_hp_func()
        .write(|w| unsafe { w.bits(0xffff_ffff) });
    pmu.hp_active_icg_hp_apb()
        .write(|w| unsafe { w.bits(0xffff_ffff) });
    pmu.hp_active_icg_modem()
        .write(|w| unsafe { w.hp_active_dig_icg_modem_code().bits(0) });
    pmu.hp_active_sysclk().modify(|_, w| unsafe {
        w.hp_active_dig_sys_clk_no_div().bit(false);
        w.hp_active_icg_sys_clock_en().bit(true);
        w.hp_active_sys_clk_slp_sel().bit(false);
        w.hp_active_icg_slp_sel().bit(false);
        w.hp_active_dig_sys_clk_sel().bits(HP_SYSCLK_XTAL)
    });
    pmu.hp_active_hp_sys_cntl().modify(|_, w| {
        w.hp_active_uart_wakeup_en().bit(false);
        w.hp_active_lp_pad_hold_all().bit(false);
        w.hp_active_hp_pad_hold_all().bit(false);
        w.hp_active_dig_pad_slp_sel().bit(false);
        w.hp_active_dig_pause_wdt().bit(false);
        w.hp_active_dig_cpu_stall().bit(false)
    });
    pmu.hp_active_bias().modify(|_, w| unsafe {
        w.hp_active_dcm_vset().bits(27);
        w.hp_active_dcm_mode().bits(1);
        w.hp_active_xpd_bias().bit(true);
        w.hp_active_dbg_atten().bits(0);
        w.hp_active_pd_cur().bit(false);
        w.sleep().bit(false)
    });
    // NOTE: do NOT reprogram the HP active regulator xpd/dbias here. The
    // bootloader already applied the efuse-calibrated active voltage
    // (get_act_hp_dbias()); overriding it with a fixed default under-volts the
    // HP domain and hangs the running CPU. IDF's pmu_hp_system_init() only
    // touches the sleep-related regulator sub-fields (pmu_init.c).
    pmu.hp_active_hp_regulator0().modify(|_, w| unsafe {
        w.hp_active_hp_regulator_slp_mem_xpd().bit(false);
        w.hp_active_hp_regulator_slp_logic_xpd().bit(false);
        w.hp_active_hp_regulator_slp_logic_dbias().bits(0)
    });
    pmu.hp_active_backup().write(|w| unsafe { w.bits(0) });
    pmu.hp_active_backup_clk()
        .write(|w| unsafe { w.bits(0xffff_ffff) });

    // ---- HP_SLEEP ----
    pmu.hp_sleep_dig_power()
        .write(|w| w.hp_sleep_dcdc_switch_pd_en().bit(true));
    pmu.hp_sleep_hp_ck_power().write(|w| unsafe {
        w.hp_sleep_i2c_iso_en().bit(true);
        w.hp_sleep_i2c_retention().bit(true);
        w.hp_sleep_xpd_pll_i2c().bits(1);
        w.hp_sleep_xpd_pll().bits(0)
    });
    pmu.hp_sleep_xtal()
        .modify(|_, w| w.hp_sleep_xpd_xtal().bit(false));

    pmu.hp_sleep_icg_hp_func().write(|w| unsafe { w.bits(0) });
    pmu.hp_sleep_icg_hp_apb().write(|w| unsafe { w.bits(0) });
    pmu.hp_sleep_icg_modem()
        .write(|w| unsafe { w.hp_sleep_dig_icg_modem_code().bits(0) });
    pmu.hp_sleep_sysclk().modify(|_, w| unsafe {
        w.hp_sleep_dig_sys_clk_no_div().bit(false);
        w.hp_sleep_icg_sys_clock_en().bit(false);
        w.hp_sleep_sys_clk_slp_sel().bit(true);
        w.hp_sleep_icg_slp_sel().bit(true);
        w.hp_sleep_dig_sys_clk_sel().bits(HP_SYSCLK_XTAL)
    });
    pmu.hp_sleep_hp_sys_cntl().modify(|_, w| {
        w.hp_sleep_uart_wakeup_en().bit(true);
        w.hp_sleep_lp_pad_hold_all().bit(false);
        w.hp_sleep_hp_pad_hold_all().bit(false);
        w.hp_sleep_dig_pad_slp_sel().bit(false);
        w.hp_sleep_dig_pause_wdt().bit(true);
        w.hp_sleep_dig_cpu_stall().bit(true)
    });
    pmu.hp_sleep_bias().modify(|_, w| unsafe {
        w.hp_sleep_dcm_vset().bits(0);
        w.hp_sleep_dcm_mode().bits(1);
        w.hp_sleep_xpd_bias().bit(false);
        w.hp_sleep_dbg_atten().bits(0);
        w.hp_sleep_pd_cur().bit(true);
        w.sleep().bit(true)
    });
    pmu.hp_sleep_hp_regulator0().modify(|_, w| unsafe {
        w.hp_sleep_hp_regulator_slp_mem_xpd().bit(false);
        w.hp_sleep_hp_regulator_slp_logic_xpd().bit(false);
        w.hp_sleep_hp_regulator_xpd().bit(true);
        w.hp_sleep_hp_regulator_slp_mem_dbias().bits(1);
        w.hp_sleep_hp_regulator_slp_logic_dbias().bits(0)
    });
    pmu.hp_sleep_backup().write(|w| unsafe { w.bits(0) });
    pmu.hp_sleep_backup_clk()
        .write(|w| unsafe { w.bits(0xffff_ffff) });

    // Some PMU initial parameter configuration (force update ICG/sysclk).
    pmu.imm_modem_icg()
        .write(|w| w.update_dig_icg_modem_en().bit(true));
    pmu.imm_sleep_sysclk()
        .write(|w| w.update_dig_icg_switch().bit(true));

    // PMU_SLEEP_PROTECT_HP_LP_SLEEP
    const PMU_SLEEP_PROTECT_HP_LP_SLEEP: u8 = 2;
    pmu.slp_wakeup_cntl3()
        .modify(|_, w| unsafe { w.sleep_prt_sel().bits(PMU_SLEEP_PROTECT_HP_LP_SLEEP) });
}

/// Configures the default per-mode LP system PMU registers (LP_ACTIVE which
/// lives in the `hp_sleep_lp_*` group, and LP_SLEEP in the `lp_sleep_*` group).
/// Mirrors `pmu_lp_system_init()` in esp-idf.
fn pmu_lp_system_init() {
    let pmu = PMU::regs();

    // ---- LP_ACTIVE (hp_sleep_lp_*) ----
    pmu.hp_sleep_lp_dig_power().write(|w| unsafe { w.bits(0) });
    pmu.hp_sleep_lp_ck_power().write(|w| {
        w.hp_sleep_xpd_lppll().bit(false);
        w.hp_sleep_xpd_xtal32k().bit(true);
        w.hp_sleep_xpd_rc32k().bit(true);
        w.hp_sleep_xpd_fosc_clk().bit(true);
        w.hp_sleep_pd_osc_clk().bit(false)
    });
    // Likewise, leave the LP active regulator voltage (xpd/dbias) as the
    // bootloader calibrated it (get_act_lp_dbias()); IDF's pmu_lp_system_init()
    // only defaults the sleep sub-fields here.
    pmu.hp_sleep_lp_regulator0().modify(|_, w| unsafe {
        w.hp_sleep_lp_regulator_slp_xpd().bit(false);
        w.hp_sleep_lp_regulator_slp_dbias().bits(0)
    });
    pmu.hp_sleep_lp_regulator1()
        .modify(|_, w| unsafe { w.hp_sleep_lp_regulator_drv_b().bits(0) });

    // ---- LP_SLEEP (lp_sleep_*) ----
    pmu.lp_sleep_lp_dig_power().write(|w| unsafe { w.bits(0) });
    pmu.lp_sleep_lp_ck_power().write(|w| unsafe { w.bits(0) });
    pmu.lp_sleep_xtal()
        .modify(|_, w| w.lp_sleep_xpd_xtal().bit(false));
    pmu.lp_sleep_bias().modify(|_, w| unsafe {
        w.lp_sleep_xpd_bias().bit(false);
        w.lp_sleep_dbg_atten().bits(0);
        w.lp_sleep_pd_cur().bit(true);
        w.sleep().bit(true)
    });
    pmu.lp_sleep_lp_regulator0().modify(|_, w| unsafe {
        w.lp_sleep_lp_regulator_slp_xpd().bit(false);
        w.lp_sleep_lp_regulator_xpd().bit(true);
        w.lp_sleep_lp_regulator_slp_dbias().bits(0);
        w.lp_sleep_lp_regulator_dbias().bits(12)
    });
    pmu.lp_sleep_lp_regulator1()
        .modify(|_, w| unsafe { w.lp_sleep_lp_regulator_drv_b().bits(0) });
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

    // 1b. Force the analog dig regulators on via regi2c (XPD off => regulated by
    //     PMU) and configure the default per-mode PMU system state (HP/LP active +
    //     sleep). Required before the PMU can perform a clean sleep/wake transition.
    //     Ref: IDF rtc_clk_init.c (esp32p4).
    regi2c::I2C_DIG_REG_FORCE_RTC_DREG.write_field(1);
    regi2c::I2C_DIG_REG_FORCE_DIG_DREG.write_field(1);
    regi2c::I2C_DIG_REG_XPD_RTC_REG.write_field(0);
    regi2c::I2C_DIG_REG_XPD_DIG_REG.write_field(0);
    pmu_hp_system_init();
    pmu_lp_system_init();

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

/// Saves and restores the CPU root clock source around sleep.
///
/// Before entering sleep the CPU root clock is switched to XTAL (the PMU runs
/// the sleep FSM from a known clock); on wake we restore the previous source.
#[derive(Clone, Copy)]
pub(crate) struct SavedClockConfig {
    old_cpu_root_clk: Option<CpuRootClkConfig>,
}

impl SavedClockConfig {
    pub(crate) fn save(clocks: &ClockTree) -> Self {
        SavedClockConfig {
            old_cpu_root_clk: clocks.cpu_root_clk(),
        }
    }

    pub(crate) fn restore(self, clocks: &mut ClockTree) {
        if let Some(old) = self.old_cpu_root_clk {
            clocks::configure_cpu_root_clk(clocks, old);
        }
    }
}

// Note: the PMU setup is based on esp-idf v5.1.2. Related code should be based
// on the same version until documentation is released and the code can be
// reasoned about.

use fugit::HertzU32;
use strum::FromRepr;

use crate::{
    clock::{
        clocks_ll::{
            esp32c6_bbpll_get_freq_mhz,
            esp32c6_cpu_get_hs_divider,
            esp32c6_cpu_get_ls_divider,
            esp32c6_rtc_bbpll_configure_raw,
            esp32c6_rtc_freq_to_pll_mhz_raw,
            esp32c6_rtc_update_to_8m,
            esp32c6_rtc_update_to_xtal_raw,
            regi2c_write_mask,
        },
        Clock,
        XtalClock,
    },
    peripherals::TIMG0,
    radio_clock_ctrl::RadioPeripherals,
    rtc_cntl::RtcClock,
    soc::efuse::Efuse,
};

const I2C_DIG_REG: u8 = 0x6d;
const I2C_DIG_REG_HOSTID: u8 = 0;

const I2C_DIG_REG_XPD_RTC_REG: u8 = 13;
const I2C_DIG_REG_XPD_RTC_REG_MSB: u8 = 2;
const I2C_DIG_REG_XPD_RTC_REG_LSB: u8 = 2;

const I2C_DIG_REG_XPD_DIG_REG: u8 = 13;
const I2C_DIG_REG_XPD_DIG_REG_MSB: u8 = 3;
const I2C_DIG_REG_XPD_DIG_REG_LSB: u8 = 3;

const I2C_DIG_REG_ENIF_RTC_DREG: u8 = 5;
const I2C_DIG_REG_ENIF_RTC_DREG_MSB: u8 = 7;
const I2C_DIG_REG_ENIF_RTC_DREG_LSB: u8 = 7;

const I2C_DIG_REG_ENIF_DIG_DREG: u8 = 7;
const I2C_DIG_REG_ENIF_DIG_DREG_MSB: u8 = 7;
const I2C_DIG_REG_ENIF_DIG_DREG_LSB: u8 = 7;

const I2C_DIG_REG_SCK_DCAP: u8 = 14;
const I2C_DIG_REG_SCK_DCAP_MSB: u8 = 7;
const I2C_DIG_REG_SCK_DCAP_LSB: u8 = 0;

unsafe fn pmu<'a>() -> &'a esp32c6::pmu::RegisterBlock {
    &*esp32c6::PMU::ptr()
}

unsafe fn modem_lpcon<'a>() -> &'a esp32c6::modem_lpcon::RegisterBlock {
    &*esp32c6::MODEM_LPCON::ptr()
}

unsafe fn modem_syscon<'a>() -> &'a esp32c6::modem_syscon::RegisterBlock {
    &*esp32c6::MODEM_SYSCON::ptr()
}

unsafe fn lp_clkrst<'a>() -> &'a esp32c6::lp_clkrst::RegisterBlock {
    &*esp32c6::LP_CLKRST::ptr()
}

unsafe fn pcr<'a>() -> &'a esp32c6::pcr::RegisterBlock {
    &*esp32c6::PCR::ptr()
}

unsafe fn lp_aon<'a>() -> &'a esp32c6::lp_aon::RegisterBlock {
    &*esp32c6::LP_AON::ptr()
}

fn pmu_power_domain_force_default() {
    unsafe {
        // for bypass reserved power domain

        // PMU_HP_PD_TOP
        pmu().power_pd_top_cntl().modify(|_, w| {
            w.force_top_reset() // pmu_ll_hp_set_power_force_reset
                .bit(false)
                .force_top_iso() // pmu_ll_hp_set_power_force_isolate
                .bit(false)
                .force_top_pu() // pmu_ll_hp_set_power_force_power_up
                .bit(false)
                .force_top_no_reset() // pmu_ll_hp_set_power_force_no_reset
                .bit(false)
                .force_top_no_iso() // pmu_ll_hp_set_power_force_no_isolate
                .bit(false)
                .force_top_pd() // pmu_ll_hp_set_power_force_power_down
                .bit(false)
        });

        // PMU_HP_PD_HP_AON
        pmu().power_pd_hpaon_cntl().modify(|_, w| {
            w.force_hp_aon_reset() // pmu_ll_hp_set_power_force_reset
                .bit(false)
                .force_hp_aon_iso() // pmu_ll_hp_set_power_force_isolate
                .bit(false)
                .force_hp_aon_pu() // pmu_ll_hp_set_power_force_power_up
                .bit(false)
                .force_hp_aon_no_reset() // pmu_ll_hp_set_power_force_no_reset
                .bit(false)
                .force_hp_aon_no_iso() // pmu_ll_hp_set_power_force_no_isolate
                .bit(false)
                .force_hp_aon_pd() // pmu_ll_hp_set_power_force_power_down
                .bit(false)
        });

        // PMU_HP_PD_CPU
        pmu().power_pd_hpcpu_cntl().modify(|_, w| {
            w.force_hp_cpu_reset() // pmu_ll_hp_set_power_force_reset
                .bit(false)
                .force_hp_cpu_iso() // pmu_ll_hp_set_power_force_isolate
                .bit(false)
                .force_hp_cpu_pu() // pmu_ll_hp_set_power_force_power_up
                .bit(false)
                .force_hp_cpu_no_reset() // pmu_ll_hp_set_power_force_no_reset
                .bit(false)
                .force_hp_cpu_no_iso() // pmu_ll_hp_set_power_force_no_isolate
                .bit(false)
                .force_hp_cpu_pd() // pmu_ll_hp_set_power_force_power_down
                .bit(false)
        });

        // PMU_HP_PD_WIFI
        pmu().power_pd_hpwifi_cntl().modify(|_, w| {
            w.force_hp_wifi_reset() // pmu_ll_hp_set_power_force_reset
                .bit(false)
                .force_hp_wifi_iso() // pmu_ll_hp_set_power_force_isolate
                .bit(false)
                .force_hp_wifi_pu() // pmu_ll_hp_set_power_force_power_up
                .bit(false)
                .force_hp_wifi_no_reset() // pmu_ll_hp_set_power_force_no_reset
                .bit(false)
                .force_hp_wifi_no_iso() // pmu_ll_hp_set_power_force_no_isolate
                .bit(false)
                .force_hp_wifi_pd() // pmu_ll_hp_set_power_force_power_down
                .bit(false)
        });

        // Isolate all memory banks while sleeping, avoid memory leakage current

        pmu().power_pd_mem_cntl().modify(|_, w| {
            w.force_hp_mem_no_iso() // pmu_ll_hp_set_memory_no_isolate
                .bits(0)
        });

        pmu().power_pd_lpperi_cntl().modify(|_, w| {
            w.force_lp_peri_reset() // pmu_ll_lp_set_power_force_reset
                .bit(false)
                .force_lp_peri_iso() // pmu_ll_lp_set_power_force_isolate
                .bit(false)
                .force_lp_peri_pu() // pmu_ll_lp_set_power_force_power_up
                .bit(false)
                .force_lp_peri_no_reset() // pmu_ll_lp_set_power_force_no_reset
                .bit(false)
                .force_lp_peri_no_iso() // pmu_ll_lp_set_power_force_no_isolate
                .bit(false)
                .force_lp_peri_pd() // pmu_ll_lp_set_power_force_power_down
                .bit(false)
        });
    };
}

fn modem_clock_domain_power_state_icg_map_init() {
    // C6 has SOC_PM_SUPPORT_PMU_MODEM_STATE defined

    // const ICG_NOGATING_SLEEP: u8 = 1 << 0; // unused
    const ICG_NOGATING_MODEM: u8 = 1 << 1;
    const ICG_NOGATING_ACTIVE: u8 = 1 << 2;

    // the ICG code's bit 0, 1 and 2 indicates the ICG state
    // of pmu SLEEP, MODEM and ACTIVE mode respectively
    unsafe {
        modem_syscon().clk_conf_power_st().modify(|_, w| {
            w.clk_modem_apb_st_map() // modem_syscon_ll_set_modem_apb_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_modem_peri_st_map() // modem_syscon_ll_set_modem_periph_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE)
                .clk_wifi_st_map() // modem_syscon_ll_set_wifi_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_bt_st_map() // modem_syscon_ll_set_bt_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_fe_st_map() // modem_syscon_ll_set_fe_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_zb_st_map() // modem_syscon_ll_set_ieee802154_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
        });

        modem_lpcon().clk_conf_power_st().modify(|_, w| {
            w.clk_lp_apb_st_map() // modem_lpcon_ll_set_lp_apb_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_i2c_mst_st_map() // modem_lpcon_ll_set_i2c_master_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_coex_st_map() // modem_lpcon_ll_set_coex_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
                .clk_wifipwr_st_map() // modem_lpcon_ll_set_wifipwr_icg_bitmap
                .bits(ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM)
        });
    }
}

enum RtcSlowClockSource {
    /// Select RC_SLOW_CLK as RTC_SLOW_CLK source
    RcSlow  = 0,

    /// Select XTAL32K_CLK as RTC_SLOW_CLK source
    XTAL32K = 1,

    /// Select RC32K_CLK as RTC_SLOW_CLK source
    RC32K   = 2,

    /// Select OSC_SLOW_CLK (external slow clock) as RTC_SLOW_CLK source
    OscSlow = 3,

    /// Invalid RTC_SLOW_CLK source
    Invalid,
}

impl RtcSlowClockSource {
    fn current() -> Self {
        // clk_ll_rtc_slow_get_src()
        let lp_clkrst = unsafe { lp_clkrst() };
        match lp_clkrst.lp_clk_conf().read().slow_clk_sel().bits() {
            0 => Self::RcSlow,
            1 => Self::XTAL32K,
            2 => Self::RC32K,
            3 => Self::OscSlow,
            _ => Self::Invalid,
        }
    }
}

#[allow(unused)]
enum ModemClockLpclkSource {
    RcSlow = 0,
    RcFast,
    MainXtal,
    RC32K,
    XTAL32K,
    EXT32K,
}

impl From<RtcSlowClockSource> for ModemClockLpclkSource {
    fn from(src: RtcSlowClockSource) -> Self {
        match src {
            RtcSlowClockSource::RcSlow => Self::RcSlow,
            RtcSlowClockSource::XTAL32K => Self::XTAL32K,
            RtcSlowClockSource::RC32K => Self::RC32K,
            RtcSlowClockSource::OscSlow => Self::EXT32K,
            _ => Self::RcSlow,
        }
    }
}

fn modem_clock_hal_deselect_all_wifi_lpclk_source() {
    unsafe {
        modem_lpcon().wifi_lp_clk_conf().modify(|_, w| {
            w.clk_wifipwr_lp_sel_osc_slow()
                .clear_bit()
                .clk_wifipwr_lp_sel_osc_fast()
                .clear_bit()
                .clk_wifipwr_lp_sel_xtal32k()
                .clear_bit()
                .clk_wifipwr_lp_sel_xtal()
                .clear_bit()
        });
    }
}

fn modem_clock_hal_select_wifi_lpclk_source(src: ModemClockLpclkSource) {
    unsafe {
        modem_lpcon().wifi_lp_clk_conf().modify(|_, w| match src {
            ModemClockLpclkSource::RcSlow => w.clk_wifipwr_lp_sel_osc_slow().set_bit(),
            ModemClockLpclkSource::RcFast => w.clk_wifipwr_lp_sel_osc_fast().set_bit(),
            ModemClockLpclkSource::MainXtal => w.clk_wifipwr_lp_sel_xtal().set_bit(),

            ModemClockLpclkSource::RC32K
            | ModemClockLpclkSource::XTAL32K
            | ModemClockLpclkSource::EXT32K => w.clk_wifipwr_lp_sel_xtal32k().set_bit(),
        });

        modem_lpcon().modem_32k_clk_conf().modify(|_, w| match src {
            ModemClockLpclkSource::RcSlow
            | ModemClockLpclkSource::RcFast
            | ModemClockLpclkSource::MainXtal => w,

            ModemClockLpclkSource::RC32K => w.clk_modem_32k_sel().bits(1),
            ModemClockLpclkSource::XTAL32K => w.clk_modem_32k_sel().bits(0),
            ModemClockLpclkSource::EXT32K => w.clk_modem_32k_sel().bits(2),
        });
    }
}

fn modem_lpcon_ll_set_wifi_lpclk_divisor_value(divider: u16) {
    unsafe {
        modem_lpcon()
            .wifi_lp_clk_conf()
            .modify(|_, w| w.clk_wifipwr_lp_div_num().bits(divider));
    }
}

fn modem_clock_hal_enable_wifipwr_clock(enable: bool) {
    unsafe {
        modem_lpcon()
            .clk_conf()
            .modify(|_, w| w.clk_wifipwr_en().bit(enable));
    }
}

fn modem_clock_select_lp_clock_source(
    periph: RadioPeripherals,
    src: ModemClockLpclkSource,
    divider: u16,
) {
    match periph {
        RadioPeripherals::Wifi => {
            modem_clock_hal_deselect_all_wifi_lpclk_source();
            modem_clock_hal_select_wifi_lpclk_source(src);
            modem_lpcon_ll_set_wifi_lpclk_divisor_value(divider);
            modem_clock_hal_enable_wifipwr_clock(true);
        }
        RadioPeripherals::Phy | RadioPeripherals::Bt | RadioPeripherals::Ieee802154 => {
            todo!("unused by setup code")
        }
    }
}

const fn hp_retention_regdma_config(dir: u8, entry: u8) -> u8 {
    (((dir) << 2) | (entry & 0x3)) & 0x7
}

const HP_CALI_DBIAS: u8 = 25;
const LP_CALI_DBIAS: u8 = 26;

const ICG_MODEM_CODE_SLEEP: u8 = 0;
const ICG_MODEM_CODE_MODEM: u8 = 1;
const ICG_MODEM_CODE_ACTIVE: u8 = 2;

const HP_SYSCLK_XTAL: u8 = 0;
const HP_SYSCLK_PLL: u8 = 1;

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.0
    pub struct HpDigPower(u32);

    pub bool, vdd_spi_pd_en, set_vdd_spi_pd_en: 21;
    pub bool, mem_dslp     , set_mem_dslp     : 22;
    pub u8,   mem_pd_en    , set_mem_pd_en    : 26, 23;
    pub bool, wifi_pd_en   , set_wifi_pd_en   : 27;
    pub bool, cpu_pd_en    , set_cpu_pd_en    : 29;
    pub bool, aon_pd_en    , set_aon_pd_en    : 30;
    pub bool, top_pd_en    , set_top_pd_en    : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.1
    pub struct HpClkPower(u32);

    pub bool, i2c_iso_en   , set_i2c_iso_en   : 26;
    pub bool, i2c_retention, set_i2c_retention: 27;
    pub bool, xpd_bb_i2c   , set_xpd_bb_i2c   : 28;
    pub bool, xpd_bbpll_i2c, set_xpd_bbpll_i2c: 29;
    pub bool, xpd_bbpll    , set_xpd_bbpll    : 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.2
    pub struct HpXtalPower(u32);

    pub bool, xpd_xtal     , set_xpd_xtal     : 31;
}

#[derive(Clone, Copy, Default)]
// pmu_sleep_power_config_t.0
pub struct HpSysPower {
    // This is a best-guess assignment of the variants in the union `pmu_hp_power_t` union
    // In esp-idf, all three fields are `pmu_hp_power_t`
    pub dig_power: HpDigPower,
    pub clk: HpClkPower,
    pub xtal: HpXtalPower,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_sys_cntl_reg_t
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
    // pmu_hp_icg_modem_reg_t
    pub struct HpIcgModem(u32);

    pub u8, code, set_code: 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_sysclk_reg_t
    pub struct HpSysclk(u32);

    pub bool, dig_sysclk_nodiv , set_dig_sysclk_nodiv : 26;
    pub bool, icg_sysclk_en    , set_icg_sysclk_en    : 27;
    pub bool, sysclk_slp_sel   , set_sysclk_slp_sel   : 28;
    pub bool, icg_slp_sel      , set_icg_slp_sel      : 29;
    pub u8,   dig_sysclk_sel   , set_dig_sysclk_sel   : 31, 30;
}

// pmu_hp_system_clock_param_t
#[derive(Clone, Copy, Default)]
struct SystemClockParam {
    icg_func: u32,
    icg_apb: u32,
    icg_modem: HpIcgModem,
    sysclk: HpSysclk,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.0
    pub struct HpAnalogBias(u32);

    pub bool, xpd_bias  , set_xpd_bias  : 25;
    pub u8,   dbg_atten , set_dbg_atten : 29, 26;
    pub bool, pd_cur    , set_pd_cur    : 30;
    pub bool, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.1
    pub struct HpAnalogRegulator0(u32);

    // Only HP_ACTIVE modem under hp system is valid
    pub u8,   lp_dbias_vol   , set_lp_dbias_vol   : 8, 4;
    // Only HP_ACTIVE modem under hp system is valid
    pub u8,   hp_dbias_vol   , set_hp_dbias_vol   : 13, 9;
    // Only HP_ACTIVE modem under hp system is valid
    pub bool, dbias_sel      , set_dbias_sel      : 14;
    // Only HP_ACTIVE modem under hp system is valid
    pub bool, dbias_init     , set_dbias_init     : 15;

    pub bool, slp_mem_xpd    , set_slp_mem_xpd    : 16;
    pub bool, slp_logic_xpd  , set_slp_logic_xpd  : 17;
    pub bool, xpd            , set_xpd            : 18;
    pub u8,   slp_mem_dbias  , set_slp_mem_dbias  : 22, 19;
    pub u8,   slp_logic_dbias, set_slp_logic_dbias: 26, 23;
    pub u8,   dbias          , set_dbias          : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.2
    pub struct HpAnalogRegulator1(u32);

    pub u32, drv_b          , set_drv_b          : 31, 8;
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
    // pmu_hp_backup_reg_t/active
    pub struct HpActiveBackup(u32);

    pub u8,   hp_sleep2active_backup_modem_clk_code, set_hp_sleep2active_backup_modem_clk_code: 5, 4;
    pub u8,   hp_modem2active_backup_modem_clk_code, set_hp_modem2active_backup_modem_clk_code: 7, 6;
    pub bool, hp_active_retention_mode             , set_hp_active_retention_mode             : 10;
    pub bool, hp_sleep2active_retention_en         , set_hp_sleep2active_retention_en         : 11;
    pub bool, hp_modem2active_retention_en         , set_hp_modem2active_retention_en         : 12;
    pub u8,   hp_sleep2active_backup_clk_sel       , set_hp_sleep2active_backup_clk_sel       : 15, 14;
    pub u8,   hp_modem2active_backup_clk_sel       , set_hp_modem2active_backup_clk_sel       : 17, 16;
    pub u8,   hp_sleep2active_backup_mode          , set_hp_sleep2active_backup_mode          : 22, 20;
    pub u8,   hp_modem2active_backup_mode          , set_hp_modem2active_backup_mode          : 25, 23;
    pub bool, hp_sleep2active_backup_en            , set_hp_sleep2active_backup_en            : 29;
    pub bool, hp_modem2active_backup_en            , set_hp_modem2active_backup_en            : 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/modem
    pub struct HpModemBackup(u32);

    pub u8,   hp_sleep2modem_backup_modem_clk_code , set_hp_sleep2modem_backup_modem_clk_code : 5, 4;
    pub bool, hp_modem_retention_mode              , set_hp_modem_retention_mode              : 10;
    pub bool, hp_sleep2modem_retention_en          , set_hp_sleep2modem_retention_en          : 11;
    pub u8,   hp_sleep2modem_backup_clk_sel        , set_hp_sleep2modem_backup_clk_sel        : 15, 14;
    pub u8,   hp_sleep2modem_backup_mode           , set_hp_sleep2modem_backup_mode           : 22, 20;
    pub bool, hp_sleep2modem_backup_en             , set_hp_sleep2modem_backup_en             : 29;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/sleep
    pub struct HpSleepBackup(u32);

    pub u8,   hp_modem2sleep_backup_modem_clk_code , set_hp_modem2sleep_backup_modem_clk_code : 7, 6;
    pub u8,   hp_active2sleep_backup_modem_clk_code, set_hp_active2sleep_backup_modem_clk_code: 9, 8;
    pub bool, hp_sleep_retention_mode              , set_hp_sleep_retention_mode              : 10;
    pub bool, hp_modem2sleep_retention_en          , set_hp_modem2sleep_retention_en          : 12;
    pub bool, hp_active2sleep_retention_en         , set_hp_active2sleep_retention_en         : 13;
    pub u8,   hp_modem2sleep_backup_clk_sel        , set_hp_modem2sleep_backup_clk_sel        : 17, 16;
    pub u8,   hp_active2sleep_backup_clk_sel       , set_hp_active2sleep_backup_clk_sel       : 19, 18;
    pub u8,   hp_modem2sleep_backup_mode           , set_hp_modem2sleep_backup_mode           : 25, 23;
    pub u8,   hp_active2sleep_backup_mode          , set_hp_active2sleep_backup_mode          : 28, 26;
    pub bool, hp_modem2sleep_backup_en             , set_hp_modem2sleep_backup_en             : 30;
    pub bool, hp_active2sleep_backup_en            , set_hp_active2sleep_backup_en            : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // custom based on `PMU_ICG_FUNC_ENA_*` bitflag constants
    pub struct HpBackupClk(u32);

    pub bool, gdma        , set_gdma         : 0;
    pub bool, spi2        , set_spi2         : 1;
    pub bool, i2s_rx      , set_i2s_rx       : 2;
    pub bool, uart0       , set_uart0        : 3;
    pub bool, uart1       , set_uart1        : 4;
    pub bool, uhci        , set_uhci         : 5;
    pub bool, usb_device  , set_usb_device   : 6;
    pub bool, i2s_tx      , set_i2s_tx       : 7;
    pub bool, regdma      , set_regdma       : 8;
    pub bool, retention   , set_retention    : 9;
    pub bool, mem_monitor , set_mem_monitor  : 10;
    pub bool, sdio_slave  , set_sdio_slave   : 11;
    pub bool, tsens       , set_tsens        : 12;
    pub bool, tg1         , set_tg1          : 13;
    pub bool, tg0         , set_tg0          : 14;
    pub bool, hpbus       , set_hpbus        : 15;
    pub bool, soc_etm     , set_soc_etm      : 16;
    pub bool, hpcore      , set_hpcore       : 17;
    pub bool, systimer    , set_systimer     : 18;
    pub bool, sec         , set_sec          : 19;
    pub bool, saradc      , set_saradc       : 20;
    pub bool, rmt         , set_rmt          : 21;
    pub bool, pwm         , set_pwm          : 22;
    pub bool, pvt_monitor , set_pvt_monitor  : 23;
    pub bool, parl_tx     , set_parl_tx      : 24;
    pub bool, parl_rx     , set_parl_rx      : 25;
    pub bool, mspi        , set_mspi         : 26;
    pub bool, ledc        , set_ledc         : 27;
    pub bool, iomux       , set_iomux        : 28;
    pub bool, i2c         , set_i2c          : 29;
    pub bool, can1        , set_can1         : 30;
    pub bool, can0        , set_can0         : 31;
}

macro_rules! hp_system_init {
    ($state:ident => $s:ident) => {
        paste::paste! {
            unsafe {
                // Default configuration of hp-system power in active, modem and sleep modes
                pmu().[<$state _dig_power >]().modify(|_, w| w.bits($s.power.dig_power.0));
                pmu().[<$state _hp_ck_power >]().modify(|_, w| w.bits($s.power.clk.0));
                pmu().[<$state _xtal >]().modify(|_, w| w
                    .[<$state _xpd_xtal >]().bit($s.power.xtal.xpd_xtal())
                );

                // Default configuration of hp-system clock in active, modem and sleep modes
                pmu().[<$state _icg_hp_func >]().write(|w| w.bits($s.clock.icg_func));
                pmu().[<$state _icg_hp_apb >]().write(|w| w.bits($s.clock.icg_apb));
                pmu().[<$state _icg_modem >]().write(|w| w
                    .[<$state _dig_icg_modem_code >]().bits($s.clock.icg_modem.code())
                );
                pmu().[<$state _sysclk >]().modify(|_, w| w
                    .[<$state _dig_sys_clk_no_div >]().bit($s.clock.sysclk.dig_sysclk_nodiv())
                    .[<$state _icg_sys_clock_en >]().bit($s.clock.sysclk.icg_sysclk_en())
                    .[<$state _sys_clk_slp_sel >]().bit($s.clock.sysclk.sysclk_slp_sel())
                    .[<$state _icg_slp_sel >]().bit($s.clock.sysclk.icg_slp_sel())
                    .[<$state _dig_sys_clk_sel >]().bits($s.clock.sysclk.dig_sysclk_sel())
                );

                // Default configuration of hp-system digital sub-system in active, modem
                // and sleep modes
                pmu().[<$state _hp_sys_cntl >]().modify(|_, w| w
                    .[<$state _uart_wakeup_en >]().bit($s.syscntl.uart_wakeup_en())
                    .[<$state _lp_pad_hold_all >]().bit($s.syscntl.lp_pad_hold_all())
                    .[<$state _hp_pad_hold_all >]().bit($s.syscntl.hp_pad_hold_all())
                    .[<$state _dig_pad_slp_sel >]().bit($s.syscntl.dig_pad_slp_sel())
                    .[<$state _dig_pause_wdt >]().bit($s.syscntl.dig_pause_wdt())
                    .[<$state _dig_cpu_stall >]().bit($s.syscntl.dig_cpu_stall())
                );

                // Default configuration of hp-system analog sub-system in active, modem and
                // sleep modes
                pmu().[<$state _bias >]().modify(|_, w| w
                    .[<$state _xpd_bias >]().bit($s.anlg.bias.xpd_bias())
                    .[<$state _dbg_atten >]().bits($s.anlg.bias.dbg_atten())
                    .[<$state _pd_cur >]().bit($s.anlg.bias.pd_cur())
                    .sleep().bit($s.anlg.bias.bias_sleep())
                );

                pmu().[<$state _hp_regulator0 >]().modify(|_, w| w
                    .[<$state _hp_regulator_slp_mem_xpd >]().bit($s.anlg.regulator0.slp_mem_xpd())
                    .[<$state _hp_regulator_slp_logic_xpd >]().bit($s.anlg.regulator0.slp_logic_xpd())
                    .[<$state _hp_regulator_xpd >]().bit($s.anlg.regulator0.xpd())
                    .[<$state _hp_regulator_slp_mem_dbias >]().bits($s.anlg.regulator0.slp_mem_dbias())
                    .[<$state _hp_regulator_slp_logic_dbias >]().bits($s.anlg.regulator0.slp_logic_dbias())
                    .[<$state _hp_regulator_dbias >]().bits($s.anlg.regulator0.dbias())
                );

                pmu().[<$state _hp_regulator1 >]().modify(|_, w| w
                    .[<$state _hp_regulator_drv_b >]().bits($s.anlg.regulator1.drv_b())
                );

                // Default configuration of hp-system retention sub-system in active, modem
                // and sleep modes
                pmu().[<$state _backup >]().write(|w| w.bits($s.retention));
                pmu().[<$state _backup_clk >]().write(|w| w.bits($s.backup_clk));
            }
        }
    };
}

struct HpSystemInit {
    power: HpSysPower,
    clock: SystemClockParam,
    syscntl: HpSysCntlReg,
    anlg: HpAnalog,
    retention: u32,
    backup_clk: u32,
}
impl HpSystemInit {
    fn active() -> Self {
        // pmu_hp_system_init_default

        let mut power = HpSysPower::default();
        power.dig_power.set_vdd_spi_pd_en(false);
        power.dig_power.set_wifi_pd_en(false);
        power.dig_power.set_cpu_pd_en(false);
        power.dig_power.set_aon_pd_en(false);
        power.dig_power.set_top_pd_en(false);
        power.dig_power.set_mem_pd_en(0);
        power.dig_power.set_mem_dslp(false);

        power.clk.set_i2c_iso_en(false);
        power.clk.set_i2c_retention(false);
        power.clk.set_xpd_bb_i2c(true);
        power.clk.set_xpd_bbpll_i2c(true);
        power.clk.set_xpd_bbpll(true);

        power.xtal.set_xpd_xtal(true);

        let mut clock = SystemClockParam {
            icg_func: 0xffffffff,
            icg_apb: 0xffffffff,
            ..SystemClockParam::default()
        };
        clock.icg_modem.set_code(ICG_MODEM_CODE_ACTIVE);
        clock.sysclk.set_dig_sysclk_nodiv(false);
        clock.sysclk.set_icg_sysclk_en(true);
        clock.sysclk.set_sysclk_slp_sel(false);
        clock.sysclk.set_icg_slp_sel(false);
        clock.sysclk.set_dig_sysclk_sel(HP_SYSCLK_XTAL);

        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_uart_wakeup_en(false);
        syscntl.set_lp_pad_hold_all(false);
        syscntl.set_hp_pad_hold_all(false);
        syscntl.set_dig_pad_slp_sel(false);
        syscntl.set_dig_pause_wdt(false);
        syscntl.set_dig_cpu_stall(false);

        // PMU_HP_ACTIVE_ANALOG_CONFIG_DEFAULT
        let mut anlg = HpAnalog::default();
        anlg.bias.set_xpd_bias(true);
        anlg.bias.set_dbg_atten(0x0);
        anlg.bias.set_pd_cur(false);
        anlg.bias.set_bias_sleep(false);

        // TODO: These 4 aren't applied currently?
        anlg.regulator0.set_lp_dbias_vol(0xD);
        anlg.regulator0.set_hp_dbias_vol(0x1C);
        anlg.regulator0.set_dbias_sel(true);
        anlg.regulator0.set_dbias_init(true);

        anlg.regulator0.set_slp_mem_xpd(false);
        anlg.regulator0.set_slp_logic_xpd(false);
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_slp_mem_dbias(0);
        anlg.regulator0.set_slp_logic_dbias(0);
        anlg.regulator0.set_dbias(HP_CALI_DBIAS);

        anlg.regulator1.set_drv_b(0);

        let mut retention = HpActiveBackup::default();
        retention.set_hp_sleep2active_backup_modem_clk_code(2);
        retention.set_hp_modem2active_backup_modem_clk_code(2);
        retention.set_hp_active_retention_mode(false);
        retention.set_hp_sleep2active_retention_en(false);
        retention.set_hp_modem2active_retention_en(false);
        retention.set_hp_sleep2active_backup_clk_sel(0);
        retention.set_hp_modem2active_backup_clk_sel(1);
        retention.set_hp_sleep2active_backup_mode(hp_retention_regdma_config(0, 0));
        retention.set_hp_modem2active_backup_mode(hp_retention_regdma_config(0, 2));
        retention.set_hp_sleep2active_backup_en(false);
        retention.set_hp_modem2active_backup_en(false);

        let mut backup_clk = HpBackupClk::default();
        backup_clk.set_regdma(true);
        backup_clk.set_tg0(true);
        backup_clk.set_tg1(true);
        backup_clk.set_hpbus(true);
        backup_clk.set_mspi(true);
        backup_clk.set_iomux(true);
        backup_clk.set_spi2(true);
        backup_clk.set_uart0(true);
        backup_clk.set_systimer(true);

        Self {
            power,
            clock,
            syscntl,
            anlg,
            retention: retention.0,
            backup_clk: backup_clk.0,
        }
    }

    fn modem() -> Self {
        let mut power = HpSysPower::default();
        power.dig_power.set_vdd_spi_pd_en(false);
        power.dig_power.set_wifi_pd_en(false);
        power.dig_power.set_cpu_pd_en(true);
        power.dig_power.set_aon_pd_en(false);
        power.dig_power.set_top_pd_en(false);
        power.dig_power.set_mem_pd_en(0);
        power.dig_power.set_mem_dslp(false);

        power.clk.set_xpd_bb_i2c(true);
        power.clk.set_xpd_bbpll_i2c(true);
        power.clk.set_xpd_bbpll(true);
        power.clk.set_i2c_iso_en(false);
        power.clk.set_i2c_retention(false);

        power.xtal.set_xpd_xtal(true);

        let mut clock = SystemClockParam {
            icg_func: 0,
            icg_apb: 0,
            ..SystemClockParam::default()
        };
        clock.icg_modem.set_code(ICG_MODEM_CODE_MODEM);
        clock.sysclk.set_dig_sysclk_nodiv(false);
        clock.sysclk.set_icg_sysclk_en(true);
        clock.sysclk.set_sysclk_slp_sel(true);
        clock.sysclk.set_icg_slp_sel(true);
        clock.sysclk.set_dig_sysclk_sel(HP_SYSCLK_PLL);

        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_uart_wakeup_en(true);
        syscntl.set_lp_pad_hold_all(false);
        syscntl.set_hp_pad_hold_all(false);
        syscntl.set_dig_pad_slp_sel(false);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_dig_cpu_stall(true);

        let mut anlg = HpAnalog::default();
        anlg.bias.set_xpd_bias(false);
        anlg.bias.set_dbg_atten(0x0);
        anlg.bias.set_pd_cur(false);
        anlg.bias.set_bias_sleep(false);

        anlg.regulator0.set_slp_mem_xpd(false);
        anlg.regulator0.set_slp_logic_xpd(false);
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_slp_mem_dbias(0);
        anlg.regulator0.set_slp_logic_dbias(0);
        anlg.regulator0.set_dbias(HP_CALI_DBIAS);

        anlg.regulator1.set_drv_b(0);

        let mut retention = HpModemBackup::default();
        retention.set_hp_sleep2modem_backup_modem_clk_code(1);
        retention.set_hp_modem_retention_mode(false);
        retention.set_hp_sleep2modem_retention_en(false);
        retention.set_hp_sleep2modem_backup_clk_sel(0);
        retention.set_hp_sleep2modem_backup_mode(hp_retention_regdma_config(0, 1));
        retention.set_hp_sleep2modem_backup_en(false);

        let mut backup_clk = HpBackupClk::default();
        backup_clk.set_regdma(true);
        backup_clk.set_tg0(true);
        backup_clk.set_tg1(true);
        backup_clk.set_hpbus(true);
        backup_clk.set_mspi(true);
        backup_clk.set_iomux(true);
        backup_clk.set_spi2(true);
        backup_clk.set_uart0(true);
        backup_clk.set_systimer(true);

        Self {
            power,
            clock,
            syscntl,
            anlg,
            retention: retention.0,
            backup_clk: backup_clk.0,
        }
    }

    fn sleep() -> Self {
        let mut power = HpSysPower::default();
        power.dig_power.set_vdd_spi_pd_en(true);
        power.dig_power.set_mem_dslp(false);
        power.dig_power.set_mem_pd_en(0);
        power.dig_power.set_wifi_pd_en(true);
        power.dig_power.set_cpu_pd_en(false);
        power.dig_power.set_aon_pd_en(false);
        power.dig_power.set_top_pd_en(false);

        power.clk.set_i2c_iso_en(true);
        power.clk.set_i2c_retention(true);
        power.clk.set_xpd_bb_i2c(true);
        power.clk.set_xpd_bbpll_i2c(false);
        power.clk.set_xpd_bbpll(false);

        power.xtal.set_xpd_xtal(false);

        let mut clock = SystemClockParam {
            icg_func: 0,
            icg_apb: 0,
            ..SystemClockParam::default()
        };
        clock.icg_modem.set_code(ICG_MODEM_CODE_SLEEP);
        clock.sysclk.set_dig_sysclk_nodiv(false);
        clock.sysclk.set_icg_sysclk_en(false);
        clock.sysclk.set_sysclk_slp_sel(true);
        clock.sysclk.set_icg_slp_sel(true);
        clock.sysclk.set_dig_sysclk_sel(HP_SYSCLK_XTAL);

        let mut anlg = HpAnalog::default();
        anlg.bias.set_xpd_bias(false);
        anlg.bias.set_dbg_atten(0x0);
        anlg.bias.set_pd_cur(false);
        anlg.bias.set_bias_sleep(false);

        anlg.regulator0.set_slp_mem_xpd(false);
        anlg.regulator0.set_slp_logic_xpd(false);
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_slp_mem_dbias(0);
        anlg.regulator0.set_slp_logic_dbias(0);
        anlg.regulator0.set_dbias(1);

        anlg.regulator1.set_drv_b(0);

        let mut retention = HpSleepBackup::default();
        retention.set_hp_modem2sleep_backup_modem_clk_code(0);
        retention.set_hp_active2sleep_backup_modem_clk_code(2);
        retention.set_hp_sleep_retention_mode(false);
        retention.set_hp_modem2sleep_retention_en(false);
        retention.set_hp_active2sleep_retention_en(false);
        retention.set_hp_modem2sleep_backup_clk_sel(0);
        retention.set_hp_active2sleep_backup_clk_sel(0);
        retention.set_hp_modem2sleep_backup_mode(hp_retention_regdma_config(1, 1));
        retention.set_hp_active2sleep_backup_mode(hp_retention_regdma_config(1, 0));
        retention.set_hp_modem2sleep_backup_en(false);
        retention.set_hp_active2sleep_backup_en(false);

        let mut backup_clk = HpBackupClk::default();
        backup_clk.set_regdma(true);
        backup_clk.set_tg0(true);
        backup_clk.set_tg1(true);
        backup_clk.set_hpbus(true);
        backup_clk.set_mspi(true);
        backup_clk.set_iomux(true);
        backup_clk.set_spi2(true);
        backup_clk.set_uart0(true);
        backup_clk.set_systimer(true);

        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_uart_wakeup_en(true);
        syscntl.set_lp_pad_hold_all(false);
        syscntl.set_hp_pad_hold_all(false);
        syscntl.set_dig_pad_slp_sel(true);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_dig_cpu_stall(true);

        Self {
            power,
            clock,
            syscntl,
            anlg,
            retention: retention.0,
            backup_clk: backup_clk.0,
        }
    }

    fn init_default() {
        let active = Self::active();
        let modem = Self::modem();
        let sleep = Self::sleep();

        hp_system_init!(hp_active => active);
        hp_system_init!(hp_modem => modem);
        hp_system_init!(hp_sleep => sleep);

        unsafe {
            // Some PMU initial parameter configuration
            pmu()
                .imm_modem_icg()
                .write(|w| w.update_dig_icg_modem_en().bit(true));
            pmu()
                .imm_sleep_sysclk()
                .write(|w| w.update_dig_icg_switch().bit(true));

            const PMU_SLEEP_PROTECT_HP_LP_SLEEP: u8 = 2;
            pmu()
                .slp_wakeup_cntl3()
                .modify(|_, w| w.sleep_prt_sel().bits(PMU_SLEEP_PROTECT_HP_LP_SLEEP));
        }
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.0
    pub struct LpDigPower(u32);

    pub u32, mem_dslp  , set_mem_dslp  : 30;
    pub u32, peri_pd_en, set_peri_pd_en: 31;

}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.1
    pub struct LpClkPower(u32);

    pub u32, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub u32, xpd_rc32k  , set_xpd_rc32k  : 29;
    pub u32, xpd_fosc   , set_xpd_fosc   : 30;
    pub u32, pd_osc     , set_pd_osc     : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.2
    pub struct LpXtalPower(u32);

    pub bool, xpd_xtal     , set_xpd_xtal     : 31;
}

#[derive(Clone, Copy, Default)]
// pmu_sleep_power_config_t.1
pub struct LpSysPower {
    // This is a best-guess assignment of the variants in the union `pmu_lp_power_t` union
    // In esp-idf, all three fields are `pmu_lp_power_t`
    pub dig_power: LpDigPower,
    pub clk_power: LpClkPower,
    pub xtal: LpXtalPower,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.0
    pub struct LpAnalogBias(u32);

    pub bool, xpd_bias  , set_xpd_bias  : 25;
    pub u8,   dbg_atten , set_dbg_atten : 29, 26;
    pub bool, pd_cur    , set_pd_cur    : 30;
    pub bool, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.1
    pub struct LpAnalogRegulator0(u32);

    pub bool, slp_xpd  , set_slp_xpd  : 21;
    pub bool, xpd      , set_xpd      : 22;
    pub u8,   slp_dbias, set_slp_dbias: 26, 23;
    pub u8,   dbias    , set_dbias    : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.2
    pub struct LpAnalogRegulator1(u32);

    pub u8, drv_b    , set_drv_b    : 31, 28;
}

#[derive(Clone, Copy, Default)]
// pmu_lp_analog_t
pub struct LpAnalog {
    pub bias: LpAnalogBias,
    pub regulator0: LpAnalogRegulator0,
    pub regulator1: LpAnalogRegulator1,
}

macro_rules! lp_system_init {
    ($state:ident => $s:ident) => {
        paste::paste! {
            unsafe {
                // Default configuration of lp-system power in active and sleep modes
                pmu().[< $state _dig_power >]().modify(|_, w| w.bits($s.dig_power.0));
                pmu().[< $state _ck_power >]().modify(|_, w| w.bits($s.clk_power.0));

                // Default configuration of lp-system analog sub-system in active and sleep modes
                pmu().[< $state _regulator0 >]().modify(|_, w| w
                    .[< $state _regulator_slp_xpd >]().bit($s.analog_regulator0.slp_xpd())
                    .[< $state _regulator_xpd >]().bit($s.analog_regulator0.xpd())
                    .[< $state _regulator_slp_dbias >]().bits($s.analog_regulator0.slp_dbias())
                    .[< $state _regulator_dbias >]().bits($s.analog_regulator0.dbias())
                );

                pmu().[< $state _regulator1 >]().modify(|_, w| w
                    .[< $state _regulator_drv_b >]().bits($s.analog_regulator1.drv_b())
                );
            }
        }
    };
}

struct LpSystemInit {
    dig_power: LpDigPower,
    clk_power: LpClkPower,
    xtal: LpXtalPower,
    bias: LpAnalogBias,
    analog_regulator0: LpAnalogRegulator0,
    analog_regulator1: LpAnalogRegulator1,
}
impl LpSystemInit {
    fn active() -> Self {
        let mut dig_power = LpDigPower::default();
        dig_power.set_peri_pd_en(false);
        dig_power.set_mem_dslp(false);

        let mut clk_power = LpClkPower::default();
        clk_power.set_xpd_xtal32k(true);
        clk_power.set_xpd_rc32k(true);
        clk_power.set_xpd_fosc(true);

        let mut analog_regulator0 = LpAnalogRegulator0::default();
        analog_regulator0.set_slp_xpd(false);
        analog_regulator0.set_xpd(true);
        analog_regulator0.set_slp_dbias(0);
        analog_regulator0.set_dbias(26);

        let mut analog_regulator1 = LpAnalogRegulator1::default();
        analog_regulator1.set_drv_b(0);

        Self {
            dig_power,
            clk_power,
            xtal: LpXtalPower::default(),
            bias: LpAnalogBias::default(),
            analog_regulator0,
            analog_regulator1,
        }
    }

    fn sleep() -> Self {
        let mut dig_power = LpDigPower::default();
        dig_power.set_mem_dslp(true);
        dig_power.set_peri_pd_en(false);

        let mut clk_power = LpClkPower::default();
        clk_power.set_xpd_xtal32k(false);
        clk_power.set_xpd_rc32k(false);
        clk_power.set_xpd_fosc(false);
        clk_power.set_pd_osc(false);

        let mut xtal = LpXtalPower::default();
        xtal.set_xpd_xtal(false);

        let mut analog_bias = LpAnalogBias::default();
        analog_bias.set_xpd_bias(false);
        analog_bias.set_dbg_atten(0);
        analog_bias.set_pd_cur(true);
        analog_bias.set_bias_sleep(true);

        let mut analog_regulator0 = LpAnalogRegulator0::default();
        analog_regulator0.set_slp_xpd(false);
        analog_regulator0.set_xpd(true);
        analog_regulator0.set_slp_dbias(0);
        analog_regulator0.set_dbias(12);

        let mut analog_regulator1 = LpAnalogRegulator1::default();
        analog_regulator1.set_drv_b(0);

        Self {
            dig_power,
            clk_power,
            xtal,
            bias: analog_bias,
            analog_regulator0,
            analog_regulator1,
        }
    }

    fn init_default() {
        let active = Self::active();
        let sleep = Self::sleep();

        lp_system_init!(hp_sleep_lp => active);
        lp_system_init!(lp_sleep_lp => sleep);

        unsafe {
            pmu()
                .lp_sleep_xtal()
                .modify(|_, w| w.lp_sleep_xpd_xtal().bit(sleep.xtal.xpd_xtal()));

            pmu().lp_sleep_bias().modify(|_, w| {
                w.lp_sleep_xpd_bias() // pmu_ll_lp_set_bias_xpd
                    .bit(sleep.bias.xpd_bias())
                    .lp_sleep_dbg_atten() // pmu_ll_lp_set_bias_dbg_atten
                    .bits(sleep.bias.dbg_atten())
                    .lp_sleep_pd_cur() // pmu_ll_lp_set_bias_pd_cur
                    .bit(sleep.bias.pd_cur())
                    .sleep() // pmu_ll_lp_set_bias_sleep
                    .bit(sleep.bias.bias_sleep())
            });
        }
    }
}

pub(crate) fn init() {
    // pmu_init()
    let pmu = unsafe { pmu() };

    pmu.rf_pwc()
        .modify(|_, w| w.perif_i2c_rstb().set_bit().xpd_perif_i2c().set_bit());

    regi2c_write_mask(
        I2C_DIG_REG,
        I2C_DIG_REG_HOSTID,
        I2C_DIG_REG_ENIF_RTC_DREG,
        I2C_DIG_REG_ENIF_RTC_DREG_MSB,
        I2C_DIG_REG_ENIF_RTC_DREG_LSB,
        1,
    );
    regi2c_write_mask(
        I2C_DIG_REG,
        I2C_DIG_REG_HOSTID,
        I2C_DIG_REG_ENIF_DIG_DREG,
        I2C_DIG_REG_ENIF_DIG_DREG_MSB,
        I2C_DIG_REG_ENIF_DIG_DREG_LSB,
        1,
    );

    regi2c_write_mask(
        I2C_DIG_REG,
        I2C_DIG_REG_HOSTID,
        I2C_DIG_REG_XPD_RTC_REG,
        I2C_DIG_REG_XPD_RTC_REG_MSB,
        I2C_DIG_REG_XPD_RTC_REG_LSB,
        0,
    );
    regi2c_write_mask(
        I2C_DIG_REG,
        I2C_DIG_REG_HOSTID,
        I2C_DIG_REG_XPD_DIG_REG,
        I2C_DIG_REG_XPD_DIG_REG_MSB,
        I2C_DIG_REG_XPD_DIG_REG_LSB,
        0,
    );

    HpSystemInit::init_default();
    LpSystemInit::init_default();

    pmu_power_domain_force_default();

    // esp_perip_clk_init()
    modem_clock_domain_power_state_icg_map_init();

    //  During system initialization, the low-power clock source of the modem
    //  (WiFi, BLE or Coexist) follows the configuration of the slow clock source
    //  of the system. If the WiFi, BLE or Coexist module needs a higher
    //  precision sleep clock (for example, the BLE needs to use the main XTAL
    //  oscillator (40 MHz) to provide the clock during the sleep process in some
    //  scenarios), the module needs to switch to the required clock source by
    //  itself.
    // TODO - WIFI-5233
    let modem_lpclk_src = ModemClockLpclkSource::from(RtcSlowClockSource::current());

    modem_clock_select_lp_clock_source(RadioPeripherals::Wifi, modem_lpclk_src, 0);
}

pub(crate) fn configure_clock() {
    assert!(matches!(RtcClock::xtal_freq(), XtalClock::RtcXtalFreq40M));

    RtcClock::set_fast_freq(RtcFastClock::RtcFastClockRcFast);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRcSlow);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        lp_aon().store1().modify(|_, w| w.bits(cal_val));
    }

    modem_clk_domain_active_state_icg_map_preinit();
}

fn modem_clk_domain_active_state_icg_map_preinit() {
    unsafe {
        // Configure modem ICG code in PMU_ACTIVE state
        pmu()
            .hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(ICG_MODEM_CODE_ACTIVE));

        // Disable clock gating for MODEM_APB, I2C_MST and LP_APB clock domains in
        // PMU_ACTIVE state
        modem_syscon()
            .clk_conf_power_st()
            .modify(|_, w| w.clk_modem_apb_st_map().bits(1 << ICG_MODEM_CODE_ACTIVE));
        modem_lpcon().clk_conf_power_st().modify(|_, w| {
            w.clk_i2c_mst_st_map()
                .bits(1 << ICG_MODEM_CODE_ACTIVE)
                .clk_lp_apb_st_map()
                .bits(1 << ICG_MODEM_CODE_ACTIVE)
        });

        // Software trigger force update modem ICG code and ICG switch
        pmu()
            .imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        pmu()
            .imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        // The following is part of rtc_clk_init

        lp_clkrst()
            .fosc_cntl()
            .modify(|_, w| w.fosc_dfreq().bits(100));
        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_SCK_DCAP,
            I2C_DIG_REG_SCK_DCAP_MSB,
            I2C_DIG_REG_SCK_DCAP_LSB,
            128,
        );
        lp_clkrst()
            .rc32k_cntl()
            .modify(|_, w| w.rc32k_dfreq().bits(700));

        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_ENIF_RTC_DREG,
            I2C_DIG_REG_ENIF_RTC_DREG_MSB,
            I2C_DIG_REG_ENIF_RTC_DREG_LSB,
            1,
        );
        regi2c_write_mask(
            I2C_DIG_REG,
            I2C_DIG_REG_HOSTID,
            I2C_DIG_REG_ENIF_DIG_DREG,
            I2C_DIG_REG_ENIF_DIG_DREG_MSB,
            I2C_DIG_REG_ENIF_DIG_DREG_LSB,
            1,
        );

        pmu()
            .hp_active_hp_regulator0()
            .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(HP_CALI_DBIAS));
        pmu()
            .hp_sleep_lp_regulator0()
            .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(LP_CALI_DBIAS));

        // clk_ll_rc_fast_tick_conf
        pcr()
            .ctrl_tick_conf()
            .modify(|_, w| w.fosc_tick_num().bits(255));
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
    /// JTAG resets CPU
    Cpu0JtagCpu   = 0x18,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
/// RTC SLOW_CLK frequency values
pub(crate) enum RtcFastClock {
    /// Select RC_FAST_CLK as RTC_FAST_CLK source
    RtcFastClockRcFast = 0,
    /// Select XTAL_D2_CLK as RTC_FAST_CLK source
    RtcFastClockXtalD2 = 1,
}

impl Clock for RtcFastClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            RtcFastClock::RtcFastClockXtalD2 => HertzU32::Hz(40_000_000 / 2), /* TODO: Is the value correct? */
            RtcFastClock::RtcFastClockRcFast => HertzU32::Hz(17_500_000),
        }
    }
}

#[allow(clippy::enum_variant_names)]
#[derive(Debug, Clone, Copy, PartialEq)]
/// RTC SLOW_CLK frequency values
pub enum RtcSlowClock {
    /// Select RC_SLOW_CLK as RTC_SLOW_CLK source
    RtcSlowClockRcSlow  = 0,
    /// Select XTAL32K_CLK as RTC_SLOW_CLK source
    RtcSlowClock32kXtal = 1,
    /// Select RC32K_CLK as RTC_SLOW_CLK source
    RtcSlowClock32kRc   = 2,
    /// Select OSC_SLOW_CLK (external slow clock) as RTC_SLOW_CLK source
    RtcSlowOscSlow      = 3,
}

impl Clock for RtcSlowClock {
    fn frequency(&self) -> HertzU32 {
        match self {
            RtcSlowClock::RtcSlowClockRcSlow => HertzU32::Hz(136_000),
            RtcSlowClock::RtcSlowClock32kXtal => HertzU32::Hz(32_768),
            RtcSlowClock::RtcSlowClock32kRc => HertzU32::Hz(32_768),
            RtcSlowClock::RtcSlowOscSlow => HertzU32::Hz(32_768),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
/// Clock source to be calibrated using rtc_clk_cal function
pub(crate) enum RtcCalSel {
    /// Currently selected RTC SLOW_CLK
    RtcCalRtcMux     = -1,
    /// Internal 150kHz RC oscillator
    RtcCalRcSlow     = 0,
    /// External 32kHz XTAL, as one type of 32k clock
    RtcCal32kXtal    = 1,
    /// Internal 32kHz RC oscillator, as one type of 32k clock
    RtcCal32kRc      = 2,
    /// External slow clock signal input by lp_pad_gpio0, as one type of 32k
    /// clock
    RtcCal32kOscSlow = 3,
    /// Internal 20MHz RC oscillator
    RtcCalRcFast,
}

#[derive(Clone)]
pub(crate) enum RtcCaliClkSel {
    CaliClkRcSlow = 0,
    CaliClkRcFast = 1,
    CaliClk32k    = 2,
}

/// RTC Watchdog Timer driver
impl RtcClock {
    // rtc_clk_xtal_freq_get
    pub(crate) fn xtal_freq_mhz() -> u32 {
        Self::read_xtal_freq_mhz().unwrap_or(40)
    }

    /// Get main XTAL frequency
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG by the
    /// bootloader, as passed to rtc_clk_init function.
    pub fn xtal_freq() -> XtalClock {
        match Self::xtal_freq_mhz() {
            40 => XtalClock::RtcXtalFreq40M,
            other => XtalClock::RtcXtalFreqOther(other),
        }
    }

    /// Get the RTC_SLOW_CLK source
    pub fn slow_freq() -> RtcSlowClock {
        let lp_clrst = unsafe { lp_clkrst() };

        let slow_freq = lp_clrst.lp_clk_conf().read().slow_clk_sel().bits();
        match slow_freq {
            0 => RtcSlowClock::RtcSlowClockRcSlow,
            1 => RtcSlowClock::RtcSlowClock32kXtal,
            2 => RtcSlowClock::RtcSlowClock32kRc,
            3 => RtcSlowClock::RtcSlowOscSlow,
            _ => unreachable!(),
        }
    }

    fn set_slow_freq(slow_freq: RtcSlowClock) {
        unsafe {
            lp_clkrst()
                .lp_clk_conf()
                .modify(|_, w| w.slow_clk_sel().bits(slow_freq as u8));

            lp_clkrst().clk_to_hp().modify(|_, w| {
                w.icg_hp_xtal32k()
                    .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock32kXtal))
                    .icg_hp_xtal32k()
                    .bit(matches!(slow_freq, RtcSlowClock::RtcSlowClock32kXtal))
            });
        }
    }

    // TODO: IDF-5781 Some of esp32c6 SOC_RTC_FAST_CLK_SRC_XTAL_D2 rtc_fast clock
    // has timing issue Force to use SOC_RTC_FAST_CLK_SRC_RC_FAST since 2nd
    // stage bootloader https://github.com/espressif/esp-idf/blob/master/components/bootloader_support/src/bootloader_clock_init.c#L65-L67
    fn set_fast_freq(fast_freq: RtcFastClock) {
        unsafe {
            lp_clkrst().lp_clk_conf().modify(|_, w| {
                w.fast_clk_sel().bit(match fast_freq {
                    RtcFastClock::RtcFastClockRcFast => false,
                    RtcFastClock::RtcFastClockXtalD2 => true,
                })
            });
        }

        crate::rom::ets_delay_us(3);
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    pub(crate) fn calibrate_internal(mut cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        const SOC_CLK_RC_FAST_FREQ_APPROX: u32 = 17_500_000;
        const SOC_CLK_RC_SLOW_FREQ_APPROX: u32 = 136_000;
        const SOC_CLK_XTAL32K_FREQ_APPROX: u32 = 32768;

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match cal_clk {
                RtcCalSel::RtcCalRtcMux => match RtcClock::slow_freq() {
                    RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                    RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                    _ => cal_clk,
                },
                RtcCalSel::RtcCal32kOscSlow => RtcCalSel::RtcCalRtcMux,
                _ => cal_clk,
            };
        }

        let lp_clkrst = unsafe { lp_clkrst() };
        let pcr = unsafe { pcr() };
        let pmu = unsafe { pmu() };

        let clk_src = RtcClock::slow_freq();

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRcSlow,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCal32kOscSlow,
            };
        }

        let cali_clk_sel;
        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match clk_src {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRcSlow,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCalRcSlow,
            }
        }

        if cal_clk == RtcCalSel::RtcCalRcFast {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcFast;
        } else if cal_clk == RtcCalSel::RtcCalRcSlow {
            cali_clk_sel = RtcCaliClkSel::CaliClkRcSlow;
        } else {
            cali_clk_sel = RtcCaliClkSel::CaliClk32k;
            match cal_clk {
                RtcCalSel::RtcCalRtcMux | RtcCalSel::RtcCalRcSlow | RtcCalSel::RtcCalRcFast => {}
                RtcCalSel::RtcCal32kRc => {
                    pcr.ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) });
                }
                RtcCalSel::RtcCal32kXtal => {
                    pcr.ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) });
                }
                RtcCalSel::RtcCal32kOscSlow => {
                    pcr.ctrl_32k_conf()
                        .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) });
                }
            }
        }

        // Enable requested clock (150k is always on)
        // Some delay is required before the time is stable
        // Only enable if originaly was disabled
        // If clock is already on, do nothing

        let dig_32k_xtal_enabled = lp_clkrst.clk_to_hp().read().icg_hp_xtal32k().bit_is_set();

        if cal_clk == RtcCalSel::RtcCal32kXtal && !dig_32k_xtal_enabled {
            lp_clkrst
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        }

        // TODO: very hacky
        // in ESP-IDF these are not called in this function but the fields are set
        lp_clkrst
            .clk_to_hp()
            .modify(|_, w| w.icg_hp_xtal32k().set_bit());
        pmu.hp_sleep_lp_ck_power()
            .modify(|_, w| w.hp_sleep_xpd_xtal32k().set_bit());

        pmu.hp_sleep_lp_ck_power()
            .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());

        let rc_fast_enabled = pmu
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_fosc_clk()
            .bit_is_set();
        let dig_rc_fast_enabled = lp_clkrst.clk_to_hp().read().icg_hp_fosc().bit_is_set();

        if cal_clk == RtcCalSel::RtcCalRcFast {
            if !rc_fast_enabled {
                pmu.hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                crate::rom::ets_delay_us(50);
            }

            if !dig_rc_fast_enabled {
                lp_clkrst
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                crate::rom::ets_delay_us(5);
            }
        }

        let rc32k_enabled = pmu
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_rc32k()
            .bit_is_set();
        let dig_rc32k_enabled = lp_clkrst.clk_to_hp().read().icg_hp_osc32k().bit_is_set();

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if !rc32k_enabled {
                pmu.hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                crate::rom::ets_delay_us(300);
            }

            if !dig_rc32k_enabled {
                lp_clkrst
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        // Check if there is already running calibration process
        // TODO: &mut TIMG0 for calibration
        let timg0 = unsafe { &*TIMG0::ptr() };

        if timg0
            .rtccalicfg()
            .read()
            .rtc_cali_start_cycling()
            .bit_is_set()
        {
            timg0
                .rtccalicfg2()
                .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

            // Set small timeout threshold to accelerate the generation of timeot
            // Internal circuit will be reset when timeout occurs and will not affect the
            // next calibration
            while !timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set()
                && !timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set()
            {}
        }

        // Prepare calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(cali_clk_sel.clone() as u8) });
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| unsafe { w.rtc_cali_max().bits(slowclk_cycles as u16) });

        let expected_freq = match cali_clk_sel {
            RtcCaliClkSel::CaliClk32k => {
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
                });
                SOC_CLK_XTAL32K_FREQ_APPROX
            }
            RtcCaliClkSel::CaliClkRcFast => {
                timg0
                    .rtccalicfg2()
                    .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(0x01FFFFFF) });
                SOC_CLK_RC_FAST_FREQ_APPROX
            }
            _ => {
                timg0.rtccalicfg2().modify(|_, w| unsafe {
                    w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
                });
                SOC_CLK_RC_SLOW_FREQ_APPROX
            }
        };

        let us_time_estimate = (HertzU32::MHz(slowclk_cycles) / expected_freq).to_Hz();

        // Start calibration
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().set_bit());

        // Wait for calibration to finish up to another us_time_estimate
        crate::rom::ets_delay_us(us_time_estimate);

        let cal_val = loop {
            if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
                // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
                // So we need to multiply the frequency of the Fosc for ECO1 and above chips by
                // 32 times. And ensure that this modification will not affect
                // ECO0.
                // https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3
                if Efuse::chip_revision() > 0 {
                    if cal_clk == RtcCalSel::RtcCalRcFast {
                        break timg0.rtccalicfg1().read().rtc_cali_value().bits() >> 5;
                    }
                    break timg0.rtccalicfg1().read().rtc_cali_value().bits();
                } else {
                    break timg0.rtccalicfg1().read().rtc_cali_value().bits();
                }
            }

            if timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
                // Timed out waiting for calibration
                break 0;
            }
        };

        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().clear_bit());

        if cal_clk == RtcCalSel::RtcCal32kXtal && !dig_32k_xtal_enabled {
            lp_clkrst
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_xtal32k().clear_bit());
        }

        if cal_clk == RtcCalSel::RtcCalRcFast {
            if rc_fast_enabled {
                pmu.hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
                crate::rom::ets_delay_us(50);
            }

            if dig_rc_fast_enabled {
                lp_clkrst
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                crate::rom::ets_delay_us(5);
            }
        }

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if rc32k_enabled {
                pmu.hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                crate::rom::ets_delay_us(300);
            }
            if dig_rc32k_enabled {
                lp_clkrst
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_osc32k().set_bit());
            }
        }

        cal_val
    }

    /// Measure RTC slow clock's period, based on main XTAL frequency
    ///
    /// This function will time out and return 0 if the time for the given
    /// number of cycles to be counted exceeds the expected time twice. This
    /// may happen if 32k XTAL is being calibrated, but the oscillator has
    /// not started up (due to incorrect loading capacitance, board design
    /// issue, or lack of 32 XTAL on board).
    fn calibrate(cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        let xtal_freq = RtcClock::xtal_freq();

        let mut slowclk_cycles = slowclk_cycles;

        // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
        // So we need to divide the calibrate cycles of the FOSC for ECO1 and above
        // chips by 32 to avoid excessive calibration time.
        if Efuse::chip_revision() > 0 && cal_clk == RtcCalSel::RtcCalRcFast {
            slowclk_cycles >>= 5;
        }

        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        let period_13q19 = RtcClock::calibrate(
            match RtcClock::slow_freq() {
                RtcSlowClock::RtcSlowClockRcSlow => RtcCalSel::RtcCalRtcMux,
                RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                RtcSlowClock::RtcSlowOscSlow => RtcCalSel::RtcCal32kOscSlow,
                // RtcSlowClock::RtcCalRcFast => RtcCalSel::RtcCalRcFast,
            },
            1024,
        );

        // 100_000_000 is used to get rid of `float` calculations
        let period = (100_000_000 * period_13q19 as u64) / (1 << RtcClock::CAL_FRACT);

        (100_000_000 * 1000 / period) as u16
    }

    pub(crate) fn estimate_xtal_frequency() -> u32 {
        let timg0 = unsafe { crate::peripherals::TIMG0::steal() };
        while timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_clear() {}

        timg0.rtccalicfg().modify(|_, w| unsafe {
            w.rtc_cali_clk_sel()
                .bits(0) // RTC_SLOW_CLK
                .rtc_cali_max()
                .bits(100)
                .rtc_cali_start_cycling()
                .clear_bit()
                .rtc_cali_start()
                .set_bit()
        });
        timg0
            .rtccalicfg()
            .modify(|_, w| w.rtc_cali_start().set_bit());

        while timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_clear() {}

        (timg0.rtccalicfg1().read().rtc_cali_value().bits()
            * (RtcSlowClock::RtcSlowClockRcSlow.frequency().to_Hz() / 100))
            / 1_000_000
    }
}

pub(crate) fn rtc_clk_cpu_freq_set_xtal() {
    // rtc_clk_cpu_set_to_default_config
    let freq = RtcClock::xtal_freq_mhz();

    esp32c6_rtc_update_to_xtal_raw(freq, 1);

    // TODO: don't turn off the bbpll if some consumers depend on bbpll
    // if !s_bbpll_digi_consumers_ref_count {
    rtc_clk_bbpll_disable();
    //}
}

#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct UnsupportedClockSource;

#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum CpuClockSource {
    Xtal,
    Pll,
    RcFast,
}

impl CpuClockSource {
    pub(crate) fn current() -> Result<Self, UnsupportedClockSource> {
        let source = match unsafe { pcr().sysclk_conf().read().soc_clk_sel().bits() } {
            0 => CpuClockSource::Xtal,
            1 => CpuClockSource::Pll,
            2 => CpuClockSource::RcFast,
            _ => return Err(UnsupportedClockSource),
        };

        Ok(source)
    }

    pub(crate) fn select(self) {
        unsafe {
            pcr().sysclk_conf().modify(|_, w| {
                w.soc_clk_sel().bits(match self {
                    CpuClockSource::Xtal => 0,
                    CpuClockSource::Pll => 1,
                    CpuClockSource::RcFast => 2,
                })
            });
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SavedClockConfig {
    /// The clock from which CPU clock is derived
    pub source: CpuClockSource,

    /// Source clock frequency
    pub source_freq_mhz: u32,

    /// Divider, freq_mhz = SOC_ROOT_CLK freq_mhz / div
    pub div: u8,
}

impl SavedClockConfig {
    pub(crate) fn save() -> Self {
        let source = unwrap!(CpuClockSource::current());

        let div;
        let source_freq_mhz;
        match source {
            CpuClockSource::Xtal => {
                div = esp32c6_cpu_get_ls_divider();
                source_freq_mhz = RtcClock::xtal_freq_mhz();
            }
            CpuClockSource::Pll => {
                div = esp32c6_cpu_get_hs_divider();
                source_freq_mhz = esp32c6_bbpll_get_freq_mhz();
            }
            CpuClockSource::RcFast => {
                div = esp32c6_cpu_get_ls_divider();
                source_freq_mhz = 20;
            }
        }

        SavedClockConfig {
            source,
            source_freq_mhz,
            div,
        }
    }

    fn freq_mhz(&self) -> u32 {
        self.source_freq_mhz / self.div as u32
    }

    // rtc_clk_cpu_freq_set_config
    pub(crate) fn restore(self) {
        let old_source = unwrap!(CpuClockSource::current());

        match self.source {
            CpuClockSource::Xtal => esp32c6_rtc_update_to_xtal_raw(self.freq_mhz(), self.div),
            CpuClockSource::RcFast => esp32c6_rtc_update_to_8m(),
            CpuClockSource::Pll => {
                if old_source != CpuClockSource::Pll {
                    rtc_clk_bbpll_enable();
                    esp32c6_rtc_bbpll_configure_raw(
                        RtcClock::xtal_freq_mhz(),
                        self.source_freq_mhz,
                    );
                }
                esp32c6_rtc_freq_to_pll_mhz_raw(self.freq_mhz());
            }
        }

        if old_source == CpuClockSource::Pll && self.source != CpuClockSource::Pll
        // && !s_bbpll_digi_consumers_ref_count
        {
            // We don't turn off the bbpll if some consumers depend on bbpll
            rtc_clk_bbpll_disable();
        }
    }
}

fn rtc_clk_bbpll_enable() {
    unsafe {
        pmu().imm_hp_ck_power().modify(|_, w| {
            w.tie_high_xpd_bb_i2c()
                .set_bit()
                .tie_high_xpd_bbpll()
                .set_bit()
                .tie_high_xpd_bbpll_i2c()
                .set_bit()
        });
        pmu()
            .imm_hp_ck_power()
            .modify(|_, w| w.tie_high_global_bbpll_icg().set_bit());
    }
}

fn rtc_clk_bbpll_disable() {
    unsafe {
        pmu()
            .imm_hp_ck_power()
            .modify(|_, w| w.tie_low_global_bbpll_icg().set_bit());

        pmu().imm_hp_ck_power().modify(|_, w| {
            w.tie_low_xpd_bbpll()
                .set_bit()
                .tie_low_xpd_bbpll_i2c()
                .set_bit()
        });
    }
}

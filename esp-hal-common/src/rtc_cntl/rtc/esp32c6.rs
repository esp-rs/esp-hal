use fugit::HertzU32;
use strum::FromRepr;

use crate::{
    clock::{clocks_ll::regi2c_write_mask, Clock, XtalClock},
    peripherals::{LP_AON, LP_CLKRST, PCR, PMU, TIMG0},
    soc::efuse::{Efuse, WAFER_VERSION_MAJOR, WAFER_VERSION_MINOR},
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

struct MachineConstants;
impl MachineConstants {
    const LP_MIN_SLP_TIME_US: u32 = 450;
    const LP_WAKEUP_WAIT_CYCLE: u32 = 4;
    const LP_ANALOG_WAIT_TIME_US: u32 = 154;
    const LP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const LP_CLK_SWITCH_CYCLE: u32 = 1;
    const LP_CLK_POWER_ON_WAIT_CYCLE: u32 = 1;
    const LP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    const LP_POWER_UP_WAIT_TIME_US: u32 = 2;

    const HP_MIN_SLP_TIME_US: u32 = 450;
    const HP_CLOCK_DOMAIN_SYNC_TIME_US: u32 = 150;
    const HP_SYSTEM_DFS_UP_WORK_TIME_US: u32 = 124;
    const HP_ANALOG_WAIT_TIME_US: u32 = 154;
    const HP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    const HP_POWER_UP_WAIT_TIME_US: u32 = 2;
    const HP_REGDMA_S2M_WORK_TIME_US: u32 = 172;
    const HP_REGDMA_S2A_WORK_TIME_US: u32 = 480;
    const HP_REGDMA_M2A_WORK_TIME_US: u32 = 278;
    // const HP_REGDMA_A2S_WORK_TIME_US: u32 = 382;
    const HP_REGDMA_RF_ON_WORK_TIME_US: u32 = 70;
    // const HP_REGDMA_RF_OFF_WORK_TIME_US: u32 = 23;
    const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const HP_PLL_WAIT_STABLE_TIME_US: u32 = 1;

    const MODEM_STATE_SKIP_TIME_US: u32 = Self::HP_REGDMA_M2A_WORK_TIME_US
        + Self::HP_SYSTEM_DFS_UP_WORK_TIME_US
        + Self::LP_MIN_SLP_TIME_US;
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
        let lp_clkrst = unsafe { &*esp32c6::LP_CLKRST::ptr() };
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
        })
    }
}

fn modem_clock_hal_select_wifi_lpclk_source(src: ModemClockLpclkSource) {
    unsafe {
        modem_lpcon().coex_lp_clk_conf().modify(|_, w| match src {
            ModemClockLpclkSource::RcSlow => w.clk_coex_lp_sel_osc_slow().set_bit(),
            ModemClockLpclkSource::RcFast => w.clk_coex_lp_sel_osc_fast().set_bit(),
            ModemClockLpclkSource::MainXtal => w.clk_coex_lp_sel_xtal().set_bit(),

            ModemClockLpclkSource::RC32K
            | ModemClockLpclkSource::XTAL32K
            | ModemClockLpclkSource::EXT32K => w.clk_coex_lp_sel_xtal32k().set_bit(),
        });

        modem_lpcon().modem_32k_clk_conf().modify(|_, w| match src {
            ModemClockLpclkSource::RcSlow
            | ModemClockLpclkSource::RcFast
            | ModemClockLpclkSource::MainXtal => w,

            ModemClockLpclkSource::RC32K => w.clk_modem_32k_sel().variant(1),
            ModemClockLpclkSource::XTAL32K => w.clk_modem_32k_sel().variant(0),
            ModemClockLpclkSource::EXT32K => w.clk_modem_32k_sel().variant(2),
        });
    }
}

fn modem_lpcon_ll_set_wifi_lpclk_divisor_value(divider: u16) {
    unsafe {
        modem_lpcon()
            .wifi_lp_clk_conf()
            .modify(|_, w| w.clk_wifipwr_lp_div_num().bits(divider))
    }
}

fn modem_clock_hal_enable_wifipwr_clock(enable: bool) {
    // FIXME: esp-idf uses refcounting here for later revisions.
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

const fn hp_retention_regdma_config(dir: u32, entry: u32) -> u32 {
    (((dir) << 2) | (entry & 0x3)) & 0x7
}

const HP_CALI_DBIAS: u32 = 25;

const ICG_MODEM_CODE_SLEEP: u32 = 0;
const ICG_MODEM_CODE_MODEM: u32 = 1;
const ICG_MODEM_CODE_ACTIVE: u32 = 2;

const HP_SYSCLK_XTAL: u32 = 0;
const HP_SYSCLK_PLL: u32 = 1;

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.0
    pub struct HpDigPower(u32);

    pub u32, reserved0    , set_reserved0    : 20, 0;
    pub u32, vdd_spi_pd_en, set_vdd_spi_pd_en: 21;
    pub u32, mem_dslp     , set_mem_dslp     : 22;
    pub u32, mem_pd_en    , set_mem_pd_en    : 26, 23;
    pub u32, wifi_pd_en   , set_wifi_pd_en   : 27;
    pub u32, reserved1    , set_reserved1    : 28;
    pub u32, cpu_pd_en    , set_cpu_pd_en    : 29;
    pub u32, aon_pd_en    , set_aon_pd_en    : 30;
    pub u32, top_pd_en    , set_top_pd_en    : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.1
    pub struct HpClkPower(u32);

    pub u32, reserved2    , set_reserved2    : 25, 0;
    pub u32, i2c_iso_en   , set_i2c_iso_en   : 26;
    pub u32, i2c_retention, set_i2c_retention: 27;
    pub u32, xpd_bb_i2c   , set_xpd_bb_i2c   : 28;
    pub u32, xpd_bbpll_i2c, set_xpd_bbpll_i2c: 29;
    pub u32, xpd_bbpll    , set_xpd_bbpll    : 30;
    pub u32, reserved3    , set_reserved3    : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.2
    pub struct HpXtalPower(u32);

    pub u32, reserved4    , set_reserved4    : 30, 0;
    pub u32, xpd_xtal     , set_xpd_xtal     : 31;
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

    pub u32, reserved0      , set_reserved0      : 23, 0;
    pub u32, uart_wakeup_en , set_uart_wakeup_en : 24;
    pub u32, lp_pad_hold_all, set_lp_pad_hold_all: 25;
    pub u32, hp_pad_hold_all, set_hp_pad_hold_all: 26;
    pub u32, dig_pad_slp_sel, set_dig_pad_slp_sel: 27;
    pub u32, dig_pause_wdt  , set_dig_pause_wdt  : 28;
    pub u32, dig_cpu_stall  , set_dig_cpu_stall  : 29;
    pub u32, reserved1      , set_reserved1      : 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_icg_modem_reg_t
    pub struct HpIcgModem(u32);

    pub u32, reserved     , set_reserved : 29, 0;
    pub u32, code         , set_code     : 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_sysclk_reg_t
    pub struct HpSysclk(u32);

    pub u32, reserved         , set_reserved         : 25, 0;
    pub u32, dig_sysclk_nodiv , set_dig_sysclk_nodiv : 26;
    pub u32, icg_sysclk_en    , set_icg_sysclk_en    : 27;
    pub u32, sysclk_slp_sel   , set_sysclk_slp_sel   : 28;
    pub u32, icg_slp_sel      , set_icg_slp_sel      : 29;
    pub u32, dig_sysclk_sel   , set_dig_sysclk_sel   : 31, 30;
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

    pub u32, reserved0 , set_reserved0 : 24, 0;
    pub u32, xpd_bias  , set_xpd_bias  : 25;
    pub u32, dbg_atten , set_dbg_atten : 29, 26;
    pub u32, pd_cur    , set_pd_cur    : 30;
    pub u32, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.1
    pub struct HpAnalogRegulator0(u32);

    pub u32, reserved1      , set_reserved1      : 3, 0;
    // Only HP_ACTIVE modem under hp system is valid
    pub u32, lp_dbias_vol   , set_lp_dbias_vol   : 8, 4;
    // Only HP_ACTIVE modem under hp system is valid
    pub u32, hp_dbias_vol   , set_hp_dbias_vol   : 13, 9;
    // Only HP_ACTIVE modem under hp system is valid
    pub u32, dbias_sel      , set_dbias_sel      : 14;
    // Only HP_ACTIVE modem under hp system is valid
    pub u32, dbias_init     , set_dbias_init     : 15;

    pub u32, slp_mem_xpd    , set_slp_mem_xpd    : 16;
    pub u32, slp_logic_xpd  , set_slp_logic_xpd  : 17;
    pub u32, xpd            , set_xpd            : 18;
    pub u32, slp_mem_dbias  , set_slp_mem_dbias  : 22, 19;
    pub u32, slp_logic_dbias, set_slp_logic_dbias: 26, 23;
    pub u32, dbias          , set_dbias          : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.2
    pub struct HpAnalogRegulator1(u32);

    pub u32, reserved2      , set_reserved2      : 7, 0;
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

    pub u32, reserved0                            , set_reserved0                            : 3, 0;
    pub u32, hp_sleep2active_backup_modem_clk_code, set_hp_sleep2active_backup_modem_clk_code: 5, 4;
    pub u32, hp_modem2active_backup_modem_clk_code, set_hp_modem2active_backup_modem_clk_code: 7, 6;
    pub u32, reserved1                            , set_reserved1                            : 9, 8;
    pub u32, hp_active_retention_mode             , set_hp_active_retention_mode             : 10;
    pub u32, hp_sleep2active_retention_en         , set_hp_sleep2active_retention_en         : 11;
    pub u32, hp_modem2active_retention_en         , set_hp_modem2active_retention_en         : 12;
    pub u32, reserved2                            , set_reserved2                            : 13;
    pub u32, hp_sleep2active_backup_clk_sel       , set_hp_sleep2active_backup_clk_sel       : 15, 14;
    pub u32, hp_modem2active_backup_clk_sel       , set_hp_modem2active_backup_clk_sel       : 17, 16;
    pub u32, reserved3                            , set_reserved3                            : 19, 18;
    pub u32, hp_sleep2active_backup_mode          , set_hp_sleep2active_backup_mode          : 22, 20;
    pub u32, hp_modem2active_backup_mode          , set_hp_modem2active_backup_mode          : 25, 23;
    pub u32, reserved4                            , set_reserved4                            : 28, 26;
    pub u32, hp_sleep2active_backup_en            , set_hp_sleep2active_backup_en            : 29;
    pub u32, hp_modem2active_backup_en            , set_hp_modem2active_backup_en            : 30;
    pub u32, reserved5                            , set_reserved5                            : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/modem
    pub struct HpModemBackup(u32);

    pub u32, reserved6                            , set_reserved6                            : 3, 0;
    pub u32, hp_sleep2modem_backup_modem_clk_code , set_hp_sleep2modem_backup_modem_clk_code : 5, 4;
    pub u32, reserved7                            , set_reserved7                            : 9, 6;
    pub u32, hp_modem_retention_mode              , set_hp_modem_retention_mode              : 10;
    pub u32, hp_sleep2modem_retention_en          , set_hp_sleep2modem_retention_en          : 11;
    pub u32, reserved8                            , set_reserved8                            : 13, 12;
    pub u32, hp_sleep2modem_backup_clk_sel        , set_hp_sleep2modem_backup_clk_sel        : 15, 14;
    pub u32, reserved9                            , set_reserved9                            : 19, 16;
    pub u32, hp_sleep2modem_backup_mode           , set_hp_sleep2modem_backup_mode           : 22, 20;
    pub u32, reserved10                           , set_reserved10                           : 28, 23;
    pub u32, hp_sleep2modem_backup_en             , set_hp_sleep2modem_backup_en             : 29;
    pub u32, reserved11                           , set_reserved11                           : 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/sleep
    pub struct HpSleepBackup(u32);

    pub u32, reserved12                           , set_reserved12                           : 5, 0;
    pub u32, hp_modem2sleep_backup_modem_clk_code , set_hp_modem2sleep_backup_modem_clk_code : 7, 6;
    pub u32, hp_active2sleep_backup_modem_clk_code, set_hp_active2sleep_backup_modem_clk_code: 9, 8;
    pub u32, hp_sleep_retention_mode              , set_hp_sleep_retention_mode              : 10;
    pub u32, reserved13                           , set_reserved13                           : 11;
    pub u32, hp_modem2sleep_retention_en          , set_hp_modem2sleep_retention_en          : 12;
    pub u32, hp_active2sleep_retention_en         , set_hp_active2sleep_retention_en         : 13;
    pub u32, reserved14                           , set_reserved14                           : 15, 14;
    pub u32, hp_modem2sleep_backup_clk_sel        , set_hp_modem2sleep_backup_clk_sel        : 17, 16;
    pub u32, hp_active2sleep_backup_clk_sel       , set_hp_active2sleep_backup_clk_sel       : 19, 18;
    pub u32, reserved15                           , set_reserved15                           : 22, 20;
    pub u32, hp_modem2sleep_backup_mode           , set_hp_modem2sleep_backup_mode           : 25, 23;
    pub u32, hp_active2sleep_backup_mode          , set_hp_active2sleep_backup_mode          : 28, 26;
    pub u32, reserved16                           , set_reserved16                           : 29;
    pub u32, hp_modem2sleep_backup_en             , set_hp_modem2sleep_backup_en             : 30;
    pub u32, hp_active2sleep_backup_en            , set_hp_active2sleep_backup_en            : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // custom based on `PMU_ICG_FUNC_ENA_*` bitflag constants
    pub struct HpBackupClk(u32);

    pub u32, gdma        , set_gdma         : 0;
    pub u32, spi2        , set_spi2         : 1;
    pub u32, i2s_rx      , set_i2s_rx       : 2;
    pub u32, uart0       , set_uart0        : 3;
    pub u32, uart1       , set_uart1        : 4;
    pub u32, uhci        , set_uhci         : 5;
    pub u32, usb_device  , set_usb_device   : 6;
    pub u32, i2s_tx      , set_i2s_tx       : 7;
    pub u32, regdma      , set_regdma       : 8;
    pub u32, retention   , set_retention    : 9;
    pub u32, mem_monitor , set_mem_monitor  : 10;
    pub u32, sdio_slave  , set_sdio_slave   : 11;
    pub u32, tsens       , set_tsens        : 12;
    pub u32, tg1         , set_tg1          : 13;
    pub u32, tg0         , set_tg0          : 14;
    pub u32, hpbus       , set_hpbus        : 15;
    pub u32, soc_etm     , set_soc_etm      : 16;
    pub u32, hpcore      , set_hpcore       : 17;
    pub u32, systimer    , set_systimer     : 18;
    pub u32, sec         , set_sec          : 19;
    pub u32, saradc      , set_saradc       : 20;
    pub u32, rmt         , set_rmt          : 21;
    pub u32, pwm         , set_pwm          : 22;
    pub u32, pvt_monitor , set_pvt_monitor  : 23;
    pub u32, parl_tx     , set_parl_tx      : 24;
    pub u32, parl_rx     , set_parl_rx      : 25;
    pub u32, mspi        , set_mspi         : 26;
    pub u32, ledc        , set_ledc         : 27;
    pub u32, iomux       , set_iomux        : 28;
    pub u32, i2c         , set_i2c          : 29;
    pub u32, can1        , set_can1         : 30;
    pub u32, can0        , set_can0         : 31;
}

struct HpSystemInit;
impl HpSystemInit {
    fn active() {
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

        let mut clock = SystemClockParam::default();
        clock.icg_func = 0xffffffff;
        clock.icg_apb = 0xffffffff;
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

        unsafe {
            // Default configuration of hp-system power in active, modem and sleep modes
            pmu()
                .hp_active_dig_power()
                .modify(|_, w| w.bits(power.dig_power.0));
            pmu()
                .hp_active_hp_ck_power()
                .modify(|_, w| w.bits(power.clk.0));
            pmu()
                .hp_active_xtal()
                .modify(|_, w| w.hp_active_xpd_xtal().bit(power.xtal.xpd_xtal()));

            // Default configuration of hp-system clock in active, modem and sleep modes
            pmu()
                .hp_active_icg_hp_func()
                .write(|w| w.bits(clock.icg_func));
            pmu()
                .hp_active_icg_hp_apb()
                .write(|w| w.bits(clock.icg_apb));
            pmu().hp_active_icg_modem().write(|w| {
                w.hp_active_dig_icg_modem_code()
                    .bits(clock.icg_modem.code() as u8)
            });
            pmu().hp_active_sysclk().modify(|_, w| {
                w.hp_active_dig_sys_clk_no_div()
                    .bit(clock.sysclk.dig_sysclk_nodiv())
                    .hp_active_icg_sys_clock_en()
                    .bit(clock.sysclk.icg_sysclk_en())
                    .hp_active_sys_clk_slp_sel()
                    .bit(clock.sysclk.sysclk_slp_sel())
                    .hp_active_icg_slp_sel()
                    .bit(clock.sysclk.icg_slp_sel())
                    .hp_active_dig_sys_clk_sel()
                    .bits(clock.sysclk.dig_sysclk_sel() as u8)
            });

            // Default configuration of hp-system digital sub-system in active, modem
            // and sleep modes
            pmu().hp_active_hp_sys_cntl().modify(|_, w| {
                w.hp_active_uart_wakeup_en()
                    .bit(syscntl.uart_wakeup_en())
                    .hp_active_lp_pad_hold_all()
                    .bit(syscntl.lp_pad_hold_all())
                    .hp_active_hp_pad_hold_all()
                    .bit(syscntl.hp_pad_hold_all())
                    .hp_active_dig_pad_slp_sel()
                    .bit(syscntl.dig_pad_slp_sel())
                    .hp_active_dig_pause_wdt()
                    .bit(syscntl.dig_pause_wdt())
                    .hp_active_dig_cpu_stall()
                    .bit(syscntl.dig_cpu_stall())
            });

            // Default configuration of hp-system analog sub-system in active, modem and
            // sleep modes
            pmu().hp_active_bias().modify(|_, w| {
                w.hp_active_xpd_bias()
                    .bit(anlg.bias.xpd_bias())
                    .hp_active_dbg_atten()
                    .bits(anlg.bias.dbg_atten() as u8)
                    .hp_active_pd_cur()
                    .bit(anlg.bias.pd_cur())
                    .sleep()
                    .bit(anlg.bias.bias_sleep())
            });

            pmu().hp_active_hp_regulator0().modify(|_, w| {
                w.hp_active_hp_regulator_slp_mem_xpd()
                    .bit(anlg.regulator0.slp_mem_xpd())
                    .hp_active_hp_regulator_slp_logic_xpd()
                    .bit(anlg.regulator0.slp_logic_xpd())
                    .hp_active_hp_regulator_xpd()
                    .bit(anlg.regulator0.xpd())
                    .hp_active_hp_regulator_slp_mem_dbias()
                    .bits(anlg.regulator0.slp_mem_dbias() as u8)
                    .hp_active_hp_regulator_slp_logic_dbias()
                    .bits(anlg.regulator0.slp_logic_dbias() as u8)
                    .hp_active_hp_regulator_dbias()
                    .bits(anlg.regulator0.dbias() as u8)
            });

            pmu().hp_active_hp_regulator1().modify(|_, w| {
                w.hp_active_hp_regulator_drv_b()
                    .bits(anlg.regulator1.drv_b())
            });

            // Default configuration of hp-system retention sub-system in active, modem
            // and sleep modes
            pmu().hp_active_backup().write(|w| w.bits(retention.0));
            pmu().hp_active_backup_clk().write(|w| w.bits(backup_clk.0));
        }
    }

    fn modem() {
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

        let mut clock = SystemClockParam::default();
        clock.icg_func = 0;
        clock.icg_apb = 0;
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

        unsafe {
            // Default configuration of hp-system power in active, modem and sleep modes
            pmu()
                .hp_modem_dig_power()
                .modify(|_, w| w.bits(power.dig_power.0));
            pmu()
                .hp_modem_hp_ck_power()
                .modify(|_, w| w.bits(power.clk.0));
            pmu()
                .hp_modem_xtal()
                .modify(|_, w| w.hp_modem_xpd_xtal().bit(power.xtal.xpd_xtal()));

            // Default configuration of hp-system clock in active, modem and sleep modes
            pmu()
                .hp_modem_icg_hp_func()
                .write(|w| w.bits(clock.icg_func));
            pmu().hp_modem_icg_hp_apb().write(|w| w.bits(clock.icg_apb));
            pmu().hp_modem_icg_modem().write(|w| {
                w.hp_modem_dig_icg_modem_code()
                    .bits(clock.icg_modem.code() as u8)
            });
            pmu().hp_modem_sysclk().modify(|_, w| {
                w.hp_modem_dig_sys_clk_no_div()
                    .bit(clock.sysclk.dig_sysclk_nodiv())
                    .hp_modem_icg_sys_clock_en()
                    .bit(clock.sysclk.icg_sysclk_en())
                    .hp_modem_sys_clk_slp_sel()
                    .bit(clock.sysclk.sysclk_slp_sel())
                    .hp_modem_icg_slp_sel()
                    .bit(clock.sysclk.icg_slp_sel())
                    .hp_modem_dig_sys_clk_sel()
                    .bits(clock.sysclk.dig_sysclk_sel() as u8)
            });

            // Default configuration of hp-system digital sub-system in active, modem
            // and sleep modes
            pmu().hp_modem_hp_sys_cntl().modify(|_, w| {
                w.hp_modem_uart_wakeup_en()
                    .bit(syscntl.uart_wakeup_en())
                    .hp_modem_lp_pad_hold_all()
                    .bit(syscntl.lp_pad_hold_all())
                    .hp_modem_hp_pad_hold_all()
                    .bit(syscntl.hp_pad_hold_all())
                    .hp_modem_dig_pad_slp_sel()
                    .bit(syscntl.dig_pad_slp_sel())
                    .hp_modem_dig_pause_wdt()
                    .bit(syscntl.dig_pause_wdt())
                    .hp_modem_dig_cpu_stall()
                    .bit(syscntl.dig_cpu_stall())
            });

            // Default configuration of hp-system analog sub-system in active, modem and
            // sleep modes
            pmu().hp_modem_bias().modify(|_, w| {
                w.hp_modem_xpd_bias()
                    .bit(anlg.bias.xpd_bias())
                    .hp_modem_dbg_atten()
                    .bits(anlg.bias.dbg_atten() as u8)
                    .hp_modem_pd_cur()
                    .bit(anlg.bias.pd_cur())
                    .sleep()
                    .bit(anlg.bias.bias_sleep())
            });

            pmu().hp_modem_hp_regulator0().modify(|_, w| {
                w.hp_modem_hp_regulator_slp_mem_xpd()
                    .bit(anlg.regulator0.slp_mem_xpd())
                    .hp_modem_hp_regulator_slp_logic_xpd()
                    .bit(anlg.regulator0.slp_logic_xpd())
                    .hp_modem_hp_regulator_xpd()
                    .bit(anlg.regulator0.xpd())
                    .hp_modem_hp_regulator_slp_mem_dbias()
                    .bits(anlg.regulator0.slp_mem_dbias() as u8)
                    .hp_modem_hp_regulator_slp_logic_dbias()
                    .bits(anlg.regulator0.slp_logic_dbias() as u8)
                    .hp_modem_hp_regulator_dbias()
                    .bits(anlg.regulator0.dbias() as u8)
            });

            pmu().hp_modem_hp_regulator1().modify(|_, w| {
                w.hp_modem_hp_regulator_drv_b()
                    .bits(anlg.regulator1.drv_b())
            });

            // Default configuration of hp-system retention sub-system in active, modem
            // and sleep modes
            pmu().hp_modem_backup().write(|w| w.bits(retention.0));
            pmu().hp_modem_backup_clk().write(|w| w.bits(backup_clk.0));
        }
    }
    fn sleep() {
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

        let mut clock = SystemClockParam::default();
        clock.icg_func = 0;
        clock.icg_apb = 0;
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

        unsafe {
            // Default configuration of hp-system power in active, modem and sleep modes
            pmu()
                .hp_sleep_dig_power()
                .modify(|_, w| w.bits(power.dig_power.0));
            pmu()
                .hp_sleep_hp_ck_power()
                .modify(|_, w| w.bits(power.clk.0));
            pmu()
                .hp_sleep_xtal()
                .modify(|_, w| w.hp_sleep_xpd_xtal().bit(power.xtal.xpd_xtal()));

            // Default configuration of hp-system clock in active, modem and sleep modes
            pmu()
                .hp_sleep_icg_hp_func()
                .write(|w| w.bits(clock.icg_func));
            pmu().hp_sleep_icg_hp_apb().write(|w| w.bits(clock.icg_apb));
            pmu().hp_sleep_icg_modem().write(|w| {
                w.hp_sleep_dig_icg_modem_code()
                    .bits(clock.icg_modem.code() as u8)
            });
            pmu().hp_sleep_sysclk().modify(|_, w| {
                w.hp_sleep_dig_sys_clk_no_div()
                    .bit(clock.sysclk.dig_sysclk_nodiv())
                    .hp_sleep_icg_sys_clock_en()
                    .bit(clock.sysclk.icg_sysclk_en())
                    .hp_sleep_sys_clk_slp_sel()
                    .bit(clock.sysclk.sysclk_slp_sel())
                    .hp_sleep_icg_slp_sel()
                    .bit(clock.sysclk.icg_slp_sel())
                    .hp_sleep_dig_sys_clk_sel()
                    .bits(clock.sysclk.dig_sysclk_sel() as u8)
            });

            // Default configuration of hp-system digital sub-system in active, modem
            // and sleep modes
            pmu().hp_sleep_hp_sys_cntl().modify(|_, w| {
                w.hp_sleep_uart_wakeup_en()
                    .bit(syscntl.uart_wakeup_en())
                    .hp_sleep_lp_pad_hold_all()
                    .bit(syscntl.lp_pad_hold_all())
                    .hp_sleep_hp_pad_hold_all()
                    .bit(syscntl.hp_pad_hold_all())
                    .hp_sleep_dig_pad_slp_sel()
                    .bit(syscntl.dig_pad_slp_sel())
                    .hp_sleep_dig_pause_wdt()
                    .bit(syscntl.dig_pause_wdt())
                    .hp_sleep_dig_cpu_stall()
                    .bit(syscntl.dig_cpu_stall())
            });

            // Default configuration of hp-system analog sub-system in active, modem and
            // sleep modes
            pmu().hp_sleep_bias().modify(|_, w| {
                w.hp_sleep_xpd_bias()
                    .bit(anlg.bias.xpd_bias())
                    .hp_sleep_dbg_atten()
                    .bits(anlg.bias.dbg_atten() as u8)
                    .hp_sleep_pd_cur()
                    .bit(anlg.bias.pd_cur())
                    .sleep()
                    .bit(anlg.bias.bias_sleep())
            });

            pmu().hp_sleep_hp_regulator0().modify(|_, w| {
                w.hp_sleep_hp_regulator_slp_mem_xpd()
                    .bit(anlg.regulator0.slp_mem_xpd())
                    .hp_sleep_hp_regulator_slp_logic_xpd()
                    .bit(anlg.regulator0.slp_logic_xpd())
                    .hp_sleep_hp_regulator_xpd()
                    .bit(anlg.regulator0.xpd())
                    .hp_sleep_hp_regulator_slp_mem_dbias()
                    .bits(anlg.regulator0.slp_mem_dbias() as u8)
                    .hp_sleep_hp_regulator_slp_logic_dbias()
                    .bits(anlg.regulator0.slp_logic_dbias() as u8)
                    .hp_sleep_hp_regulator_dbias()
                    .bits(anlg.regulator0.dbias() as u8)
            });

            pmu().hp_sleep_hp_regulator1().modify(|_, w| {
                w.hp_sleep_hp_regulator_drv_b()
                    .bits(anlg.regulator1.drv_b())
            });

            // Default configuration of hp-system retention sub-system in active, modem
            // and sleep modes
            pmu().hp_sleep_backup().write(|w| w.bits(retention.0));
            pmu().hp_sleep_backup_clk().write(|w| w.bits(backup_clk.0));
        }
    }

    fn init_default() {
        Self::active();
        Self::modem();
        Self::sleep();

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

    pub u32, reserved0 , set_reserved0 : 29, 0;
    pub u32, mem_dslp  , set_mem_dslp  : 30;
    pub u32, peri_pd_en, set_peri_pd_en: 31;

}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.1
    pub struct LpClkPower(u32);

    pub u32, reserved1  , set_reserved1  : 27, 0;
    pub u32, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub u32, xpd_rc32k  , set_xpd_rc32k  : 29;
    pub u32, xpd_fosc   , set_xpd_fosc   : 30;
    pub u32, pd_osc     , set_pd_osc     : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.2
    pub struct LpXtalPower(u32);

    pub u32, reserved2    , set_reserved2    : 30, 0;
    pub u32, xpd_xtal     , set_xpd_xtal     : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.0
    pub struct LpAnalogBias(u32);

    pub u32, reserved0 , set_reserved0 : 24, 0;
    pub u32, xpd_bias  , set_xpd_bias  : 25;
    pub u32, dbg_atten , set_dbg_atten : 29, 26;
    pub u32, pd_cur    , set_pd_cur    : 30;
    pub u32, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.1
    pub struct LpAnalogRegulator0(u32);

    pub u32, reserved1, set_reserved1: 20, 0;
    pub u32, slp_xpd  , set_slp_xpd  : 21;
    pub u32, xpd      , set_xpd      : 22;
    pub u32, slp_dbias, set_slp_dbias: 26, 23;
    pub u32, dbias    , set_dbias    : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_analog_t.2
    pub struct LpAnalogRegulator1(u32);

    pub u32, reserved2, set_reserved2: 27, 0;
    pub u32, drv_b    , set_drv_b    : 31, 28;
}

#[derive(Clone, Copy, Default)]
// pmu_lp_analog_t
pub struct LpAnalog {
    pub bias: LpAnalogBias,
    pub regulator0: LpAnalogRegulator0,
    pub regulator1: LpAnalogRegulator1,
}

struct LpSystemInit;
impl LpSystemInit {
    fn active() {
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

        unsafe {
            // Default configuration of lp-system power in active and sleep modes
            pmu()
                .hp_sleep_lp_dig_power()
                .modify(|_, w| w.bits(dig_power.0));
            pmu()
                .hp_sleep_lp_ck_power()
                .modify(|_, w| w.bits(clk_power.0));

            pmu().hp_sleep_lp_regulator0().modify(|_, w| {
                w.hp_sleep_lp_regulator_slp_xpd() // pmu_ll_hp_set_regulator_slp_xpd
                    .bit(analog_regulator0.slp_xpd())
                    .hp_sleep_lp_regulator_xpd() // pmu_ll_hp_set_regulator_xpd
                    .bit(analog_regulator0.xpd())
                    .hp_sleep_lp_regulator_slp_dbias() // pmu_ll_hp_set_regulator_sleep_dbias
                    .bits(analog_regulator0.slp_dbias() as u8)
                    .hp_sleep_lp_regulator_dbias() // pmu_ll_hp_set_regulator_dbias
                    .bits(analog_regulator0.dbias() as u8)
            });

            pmu().hp_sleep_lp_regulator1().modify(|_, w| {
                w.hp_sleep_lp_regulator_drv_b() // pmu_ll_hp_set_regulator_driver_bar
                    .bits(analog_regulator1.drv_b() as u8)
            });
        }
    }

    fn sleep() {
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

        unsafe {
            // Default configuration of lp-system power in active and sleep modes
            pmu()
                .lp_sleep_lp_dig_power()
                .modify(|_, w| w.bits(dig_power.0));
            pmu()
                .lp_sleep_lp_ck_power()
                .modify(|_, w| w.bits(clk_power.0));
            pmu()
                .lp_sleep_xtal()
                .modify(|_, w| w.lp_sleep_xpd_xtal().bit(xtal.xpd_xtal()));

            // Default configuration of lp-system analog sub-system in active and sleep
            // modes

            pmu().lp_sleep_bias().modify(|_, w| {
                w.lp_sleep_xpd_bias() // pmu_ll_lp_set_bias_xpd
                    .bit(analog_bias.xpd_bias())
                    .lp_sleep_dbg_atten() // pmu_ll_lp_set_bias_dbg_atten
                    .bits(analog_bias.dbg_atten() as u8)
                    .lp_sleep_pd_cur() // pmu_ll_lp_set_bias_pd_cur
                    .bit(analog_bias.pd_cur())
                    .sleep() // pmu_ll_lp_set_bias_sleep
                    .bit(analog_bias.bias_sleep())
            });

            pmu().lp_sleep_lp_regulator0().modify(|_, w| {
                w.lp_sleep_lp_regulator_slp_xpd() // pmu_ll_lp_set_regulator_slp_xpd
                    .bit(analog_regulator0.slp_xpd())
                    .lp_sleep_lp_regulator_xpd() // pmu_ll_lp_set_regulator_xpd
                    .bit(analog_regulator0.xpd())
                    .lp_sleep_lp_regulator_slp_dbias() // pmu_ll_lp_set_regulator_sleep_dbias
                    .bits(analog_regulator0.slp_dbias() as u8)
                    .lp_sleep_lp_regulator_dbias() // pmu_ll_lp_set_regulator_dbias
                    .bits(analog_regulator0.dbias() as u8)
            });

            pmu().lp_sleep_lp_regulator1().modify(|_, w| {
                w.lp_sleep_lp_regulator_drv_b() // pmu_ll_lp_set_regulator_driver_bar
                    .bits(analog_regulator1.drv_b() as u8)
            });
        }
    }

    fn init_default() {
        Self::active();
        Self::sleep();
    }
}

pub(crate) fn init() {
    let pmu = unsafe { &*PMU::ptr() };

    pmu.rf_pwc()
        .modify(|_, w| w.perif_i2c_rstb().set_bit().xpd_perif_i2c().set_bit());

    unsafe {
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

        pmu.hp_active_hp_regulator0()
            .modify(|_, w| w.hp_active_hp_regulator_dbias().bits(25));
        pmu.hp_sleep_lp_regulator0()
            .modify(|_, w| w.hp_sleep_lp_regulator_dbias().bits(26));
    }

    // Complete setup done by `pmu_init()`
    HpSystemInit::init_default();
    LpSystemInit::init_default();
    pmu_power_domain_force_default();

    // Follow the implementation of `esp_perip_clk_init()`
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
    assert!(matches!(
        RtcClock::get_xtal_freq(),
        XtalClock::RtcXtalFreq40M
    ));

    RtcClock::set_fast_freq(RtcFastClock::RtcFastClockRcFast);

    let cal_val = loop {
        RtcClock::set_slow_freq(RtcSlowClock::RtcSlowClockRcSlow);

        let res = RtcClock::calibrate(RtcCalSel::RtcCalRtcMux, 1024);
        if res != 0 {
            break res;
        }
    };

    unsafe {
        let lp_aon = &*LP_AON::ptr();
        lp_aon.store1().modify(|_, w| w.bits(cal_val));
    }

    modem_clk_domain_active_state_icg_map_preinit();
}

fn modem_clk_domain_active_state_icg_map_preinit() {
    unsafe {
        let pmu = &*PMU::PTR;
        let lp_clkrst = &*LP_CLKRST::PTR;
        let pcr = &*PCR::PTR;

        pmu.hp_active_icg_modem()
            .modify(|_, w| w.hp_active_dig_icg_modem_code().bits(2));

        // TODO: clean this up
        const MODEM_SYSCON_CLK_CONF_POWER_ST: u32 = 0x600A9800 + 0xc;
        const MODEM_LPCON_CLK_CONF_POWER_ST: u32 = 0x600A9800 + 0x20;

        (MODEM_SYSCON_CLK_CONF_POWER_ST as *mut u32).write_volatile(
            (MODEM_SYSCON_CLK_CONF_POWER_ST as *mut u32).read_volatile() & !(3 << 28) | 2 << 28,
        );

        (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).write_volatile(
            (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).read_volatile() & !(3 << 28) | 2 << 28,
        );

        (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).write_volatile(
            (MODEM_LPCON_CLK_CONF_POWER_ST as *mut u32).read_volatile() & !(3 << 28) | 2 << 28,
        );

        pmu.imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        pmu.imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        lp_clkrst
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
        lp_clkrst
            .rc32k_cntl()
            .modify(|_, w| w.rc32k_dfreq().bits(100));

        // https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3#diff-cc84d279f2f3d77fe252aa40a64d4813f271a52b5a4055e876efd012d888e135R810-R815
        pcr.ctrl_tick_conf()
            .modify(|_, w| w.fosc_tick_num().bits(255 as u8));
    }
}

// Terminology:
//
// CPU Reset:    Reset CPU core only, once reset done, CPU will execute from
//               reset vector
// Core Reset:   Reset the whole digital system except RTC sub-system
// System Reset: Reset the whole digital system, including RTC sub-system
// Chip Reset:   Reset the whole chip, including the analog part

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

extern "C" {
    fn ets_delay_us(us: u32);
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

#[derive(Debug, Clone, Copy, PartialEq)]
/// RTC SLOW_CLK frequency values
pub(crate) enum RtcSlowClock {
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
/// RTC Watchdog Timer
pub struct RtcClock;

/// RTC Watchdog Timer driver
impl RtcClock {
    const CAL_FRACT: u32 = 19;

    /// Get main XTAL frequency
    /// This is the value stored in RTC register RTC_XTAL_FREQ_REG by the
    /// bootloader, as passed to rtc_clk_init function.
    fn get_xtal_freq() -> XtalClock {
        let xtal_freq_reg = unsafe { &*LP_AON::PTR }.store4().read().bits();

        // Values of RTC_XTAL_FREQ_REG and RTC_APB_FREQ_REG are stored as two copies in
        // lower and upper 16-bit halves. These are the routines to work with such a
        // representation.
        let clk_val_is_valid = |val| {
            (val & 0xffffu32) == ((val >> 16u32) & 0xffffu32) && val != 0u32 && val != u32::MAX
        };
        let reg_val_to_clk_val = |val| val & u16::MAX as u32;

        if !clk_val_is_valid(xtal_freq_reg) {
            return XtalClock::RtcXtalFreq40M;
        }

        match reg_val_to_clk_val(xtal_freq_reg) {
            40 => XtalClock::RtcXtalFreq40M,
            other => XtalClock::RtcXtalFreqOther(other),
        }
    }

    /// Get the RTC_SLOW_CLK source
    pub(crate) fn get_slow_freq() -> RtcSlowClock {
        let lp_clrst = unsafe { &*LP_CLKRST::ptr() };

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
            let lp_clkrst = &*LP_CLKRST::PTR;

            lp_clkrst
                .lp_clk_conf()
                .modify(|_, w| w.slow_clk_sel().bits(slow_freq as u8));
            lp_clkrst.clk_to_hp().modify(|_, w| {
                w.icg_hp_xtal32k()
                    .bit(match slow_freq {
                        RtcSlowClock::RtcSlowClock32kXtal => true,
                        _ => false,
                    })
                    .icg_hp_xtal32k()
                    .bit(match slow_freq {
                        RtcSlowClock::RtcSlowClock32kXtal => true,
                        _ => false,
                    })
            });
        }
    }

    // TODO: IDF-5781 Some of esp32c6 SOC_RTC_FAST_CLK_SRC_XTAL_D2 rtc_fast clock
    // has timing issue Force to use SOC_RTC_FAST_CLK_SRC_RC_FAST since 2nd
    // stage bootloader https://github.com/espressif/esp-idf/blob/master/components/bootloader_support/src/bootloader_clock_init.c#L65-L67
    fn set_fast_freq(fast_freq: RtcFastClock) {
        unsafe {
            let lp_clkrst = &*LP_CLKRST::PTR;
            lp_clkrst.lp_clk_conf().modify(|_, w| {
                w.fast_clk_sel().bit(match fast_freq {
                    RtcFastClock::RtcFastClockRcFast => false,
                    RtcFastClock::RtcFastClockXtalD2 => true,
                })
            });
            ets_delay_us(3);
        }
    }

    /// Calibration of RTC_SLOW_CLK is performed using a special feature of
    /// TIMG0. This feature counts the number of XTAL clock cycles within a
    /// given number of RTC_SLOW_CLK cycles.
    fn calibrate_internal(mut cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
        const SOC_CLK_RC_FAST_FREQ_APPROX: u32 = 17_500_000;
        const SOC_CLK_RC_SLOW_FREQ_APPROX: u32 = 136_000;
        const SOC_CLK_XTAL32K_FREQ_APPROX: u32 = 32768;

        if cal_clk == RtcCalSel::RtcCalRtcMux {
            cal_clk = match cal_clk {
                RtcCalSel::RtcCalRtcMux => match RtcClock::get_slow_freq() {
                    RtcSlowClock::RtcSlowClock32kXtal => RtcCalSel::RtcCal32kXtal,
                    RtcSlowClock::RtcSlowClock32kRc => RtcCalSel::RtcCal32kRc,
                    _ => cal_clk,
                },
                RtcCalSel::RtcCal32kOscSlow => RtcCalSel::RtcCalRtcMux,
                _ => cal_clk,
            };
        }

        let lp_clkrst = unsafe { &*LP_CLKRST::ptr() };
        let pcr = unsafe { &*PCR::ptr() };
        let pmu = unsafe { &*PMU::ptr() };

        let clk_src = RtcClock::get_slow_freq();

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
                RtcCalSel::RtcCalRtcMux | RtcCalSel::RtcCalRcSlow | RtcCalSel::RtcCalRcFast => (),
                RtcCalSel::RtcCal32kRc => pcr
                    .ctrl_32k_conf()
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) }),
                RtcCalSel::RtcCal32kXtal => pcr
                    .ctrl_32k_conf()
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) }),
                RtcCalSel::RtcCal32kOscSlow => pcr
                    .ctrl_32k_conf()
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) }),
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
                unsafe {
                    ets_delay_us(50);
                }
            }

            if !dig_rc_fast_enabled {
                lp_clkrst
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                unsafe {
                    ets_delay_us(5);
                }
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
                unsafe {
                    ets_delay_us(300);
                }
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
        unsafe {
            ets_delay_us(us_time_estimate);
        }

        let cal_val = loop {
            if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
                let minor: u8 = Efuse::read_field_le(WAFER_VERSION_MINOR);
                let major: u8 = Efuse::read_field_le(WAFER_VERSION_MAJOR);

                // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
                // So we need to multiply the frequency of the Fosc for ECO1 and above chips by
                // 32 times. And ensure that this modification will not affect
                // ECO0. PS: For ESP32C6 ECO0 chip version is v0.0 only, which
                // means that both MAJOR and MINOR are 0. The chip version is
                // calculated using the following formula: MAJOR * 100 + MINOR. (if the result
                // is 1, then version is v0.1) https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3
                if (major * 100 + minor) > 0 {
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
                unsafe {
                    ets_delay_us(50);
                }
            }

            if dig_rc_fast_enabled {
                lp_clkrst
                    .clk_to_hp()
                    .modify(|_, w| w.icg_hp_fosc().set_bit());
                unsafe {
                    ets_delay_us(5);
                }
            }
        }

        if cal_clk == RtcCalSel::RtcCal32kRc {
            if rc32k_enabled {
                pmu.hp_sleep_lp_ck_power()
                    .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                unsafe {
                    ets_delay_us(300);
                }
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
        let xtal_freq = RtcClock::get_xtal_freq();

        let minor: u8 = Efuse::read_field_le(WAFER_VERSION_MINOR);
        let major: u8 = Efuse::read_field_le(WAFER_VERSION_MAJOR);

        let mut slowclk_cycles = slowclk_cycles;

        // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
        // So we need to divide the calibrate cycles of the FOSC for ECO1 and above
        // chips by 32 to avoid excessive calibration time.*/
        // PS: For ESP32C6 ECO0 chip version is v0.0 only, which means that both MAJOR
        // and MINOR are 0. The chip version is calculated using the following
        // formula: MAJOR * 100 + MINOR. (if the result is 1, then version is v0.1)
        if (major * 100 + minor) > 0 {
            if cal_clk == RtcCalSel::RtcCalRcFast {
                slowclk_cycles >>= 5;
            }
        }

        let xtal_cycles = RtcClock::calibrate_internal(cal_clk, slowclk_cycles) as u64;
        let divider = xtal_freq.mhz() as u64 * slowclk_cycles as u64;
        let period_64 = ((xtal_cycles << RtcClock::CAL_FRACT) + divider / 2u64 - 1u64) / divider;

        (period_64 & u32::MAX as u64) as u32
    }

    /// Calculate the necessary RTC_SLOW_CLK cycles to complete 1 millisecond.
    pub(crate) fn cycles_to_1ms() -> u16 {
        let period_13q19 = RtcClock::calibrate(
            match RtcClock::get_slow_freq() {
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
}

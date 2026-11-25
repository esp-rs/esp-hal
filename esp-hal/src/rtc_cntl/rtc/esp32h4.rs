use strum::FromRepr;

use crate::{
    peripherals::{LP_AON, LP_CLKRST, PMU},
    soc::{clocks::ClockConfig, regi2c, xtal32k},
};

// IDF sourced values.
const CK8M_DFREQ: u16 = 100;
const SCK_DCAP: u16 = 28;
pub(crate) const HP_CALI_DRVB: u32 = 6;
pub(crate) const LP_CALI_DBIAS: u8 = 0;

const ICG_MODEM_CODE_SLEEP: u8 = 0;
const ICG_MODEM_CODE_MODEM: u8 = 1;
const ICG_MODEM_CODE_ACTIVE: u8 = 2;

// `PMU_CLK_SRC_VAL` of esp-idf `pmu_param.c`, used by `dig_sysclk_sel` and `backup_clk_sel`.
const PMU_CLK_SRC_XTAL: u8 = 0;
const PMU_CLK_SRC_XTAL_X2: u8 = 2;
const PMU_CLK_SRC_PLL: u8 = 3;

// `PMU_ICG_FUNC_ENA_HPBUS` of esp-idf `pmu_icg_mapping.h`.
const ICG_FUNC_ENA_HPBUS: u32 = 1 << 15;

const fn hp_retention_regdma_config(dir: u8, entry: u8) -> u8 {
    (((dir) << 4) | (entry & 0xf)) & 0x1f
}

/// Clears the force bits of the power domains.
fn pmu_power_domain_force_default() {
    // PMU_HP_PD_TOP
    PMU::regs().power_pd_top_cntl().modify(|_, w| {
        w.force_top_reset().bit(false); // pmu_ll_hp_set_power_force_reset
        w.force_top_iso().bit(false); // pmu_ll_hp_set_power_force_isolate
        w.force_top_pu().bit(false); // pmu_ll_hp_set_power_force_power_up
        w.force_top_no_reset().bit(false); // pmu_ll_hp_set_power_force_no_reset
        w.force_top_no_iso().bit(false); // pmu_ll_hp_set_power_force_no_isolate
        w.force_top_pd().bit(false) // pmu_ll_hp_set_power_force_power_down
    });

    // PMU_HP_PD_HP_AON
    PMU::regs().power_pd_hpaon_cntl().modify(|_, w| {
        w.force_hp_aon_reset().bit(false);
        w.force_hp_aon_iso().bit(false);
        w.force_hp_aon_pu().bit(false);
        w.force_hp_aon_no_reset().bit(false);
        w.force_hp_aon_no_iso().bit(false);
        w.force_hp_aon_pd().bit(false)
    });

    // PMU_HP_PD_CPU
    PMU::regs().power_pd_hpcpu_cntl().modify(|_, w| {
        w.force_hp_cpu_reset().bit(false);
        w.force_hp_cpu_iso().bit(false);
        w.force_hp_cpu_pu().bit(false);
        w.force_hp_cpu_no_reset().bit(false);
        w.force_hp_cpu_no_iso().bit(false);
        w.force_hp_cpu_pd().bit(false)
    });

    // PMU_HP_PD_BT_154
    PMU::regs().power_pd_hpwifi_cntl().modify(|_, w| {
        w.force_hp_wifi_reset().bit(false);
        w.force_hp_wifi_iso().bit(false);
        w.force_hp_wifi_pu().bit(false);
        w.force_hp_wifi_no_reset().bit(false);
        w.force_hp_wifi_no_iso().bit(false);
        w.force_hp_wifi_pd().bit(false)
    });

    PMU::regs().power_pd_mem_cntl().modify(|_, w| unsafe {
        // Isolate all memory banks while sleeping, avoid memory leakage current
        w.force_hp_mem_no_iso().bits(0); // pmu_ll_hp_set_memory_no_isolate
        // Disable memory force pu for memory pd during deep sleep
        w.force_hp_mem_pu().bits(0) // pmu_ll_hp_set_memory_power_up
    });

    // pmu_ll_hp_set_vdd_flash_tiel_enable
    PMU::regs().power_vdd_flash().modify(|_, w| {
        w.flash_ldo_tiel_en().set_bit();
        w.flash_ldo_tiel().set_bit()
    });

    PMU::regs().power_pd_lpperi_cntl().modify(|_, w| {
        w.force_lp_peri_reset().bit(false); // pmu_ll_lp_set_power_force_reset
        w.force_lp_peri_iso().bit(false); // pmu_ll_lp_set_power_force_isolate
        w.force_lp_peri_pu().bit(false); // pmu_ll_lp_set_power_force_power_up
        w.force_lp_peri_no_reset().bit(false); // pmu_ll_lp_set_power_force_no_reset
        w.force_lp_peri_no_iso().bit(false); // pmu_ll_lp_set_power_force_no_isolate
        w.force_lp_peri_pd().bit(false) // pmu_ll_lp_set_power_force_power_down
    });
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_power_t.0
    pub struct HpDigPower(u32);

    pub u8,   vdd_flash_mode, set_vdd_flash_mode: 21, 18;
    pub bool, mem_dslp      , set_mem_dslp      : 22;
    pub u8,   mem_pd_en     , set_mem_pd_en     : 26, 23;
    pub bool, wifi_pd_en    , set_wifi_pd_en    : 27;
    pub bool, peri_pd_en    , set_peri_pd_en    : 28;
    pub bool, cpu_pd_en     , set_cpu_pd_en     : 29;
    pub bool, aon_pd_en     , set_aon_pd_en     : 30;
    pub bool, top_pd_en     , set_top_pd_en     : 31;
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
    pub struct XtalPower(u32);

    pub bool, xpd_xtalx2, set_xpd_xtalx2: 30;
    pub bool, xpd_xtal  , set_xpd_xtal  : 31;
}

#[derive(Clone, Copy, Default)]
// pmu_sleep_power_config_t.hp_sys
pub struct HpSysPower {
    pub dig_power: HpDigPower,
    pub clk: HpClkPower,
    pub xtal: XtalPower,
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
    // pmu_hp_sysclk_reg_t
    pub struct HpSysclk(u32);

    pub bool, dig_sysclk_nodiv, set_dig_sysclk_nodiv: 26;
    pub bool, icg_sysclk_en   , set_icg_sysclk_en   : 27;
    pub bool, sysclk_slp_sel  , set_sysclk_slp_sel  : 28;
    pub bool, icg_slp_sel     , set_icg_slp_sel     : 29;
    pub u8,   dig_sysclk_sel  , set_dig_sysclk_sel  : 31, 30;
}

// pmu_hp_system_clock_param_t
#[derive(Clone, Copy, Default)]
struct SystemClockParam {
    icg_func: u32,
    icg_apb: u32,
    icg_modem_code: u8,
    sysclk: HpSysclk,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.0, which is also pmu_lp_analog_t.0 without `xpd_trx`
    pub struct AnalogBias(u32);

    pub bool, dcdc_ccm_enb      , set_dcdc_ccm_enb      : 9;
    pub bool, dcdc_clear_rdy    , set_dcdc_clear_rdy    : 10;
    pub u8,   dig_reg_dpcur_bias, set_dig_reg_dpcur_bias: 12, 11;
    pub u8,   dig_reg_dsfmos    , set_dig_reg_dsfmos    : 16, 13;
    pub u8,   dcm_vset          , set_dcm_vset          : 21, 17;
    pub u8,   dcm_mode          , set_dcm_mode          : 23, 22;
    pub bool, xpd_trx           , set_xpd_trx           : 24;
    pub bool, xpd_bias          , set_xpd_bias          : 25;
    pub bool, discnnt_dig_rtc   , set_discnnt_dig_rtc   : 29;
    pub bool, pd_cur            , set_pd_cur            : 30;
    pub bool, bias_sleep        , set_bias_sleep        : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.1
    pub struct HpAnalogRegulator0(u32);

    pub bool, power_det_bypass, set_power_det_bypass: 0;
    // Only the HP_ACTIVE mode has these two fields.
    pub bool, dbias_sel       , set_dbias_sel       : 14;
    pub bool, dbias_init      , set_dbias_init      : 15;
    pub bool, slp_mem_xpd     , set_slp_mem_xpd     : 16;
    pub bool, slp_logic_xpd   , set_slp_logic_xpd   : 17;
    pub bool, xpd             , set_xpd             : 18;
    pub u8,   slp_mem_dbias   , set_slp_mem_dbias   : 22, 19;
    pub u8,   slp_logic_dbias , set_slp_logic_dbias : 26, 23;
    pub u8,   dbias           , set_dbias           : 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_analog_t.2
    pub struct HpAnalogRegulator1(u32);

    pub u32, drv_b, set_drv_b: 31, 8;
}

#[derive(Clone, Copy, Default)]
// pmu_hp_analog_t
pub struct HpAnalog {
    pub bias: AnalogBias,
    pub regulator0: HpAnalogRegulator0,
    pub regulator1: HpAnalogRegulator1,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/active
    struct HpActiveBackup(u32);

    pub u8,   hp_sleep2active_backup_modem_clk_code, set_hp_sleep2active_backup_modem_clk_code: 5, 4;
    pub u8,   hp_modem2active_backup_modem_clk_code, set_hp_modem2active_backup_modem_clk_code: 7, 6;
    pub u8,   hp_sleep2active_backup_clk_sel       , set_hp_sleep2active_backup_clk_sel       : 15, 14;
    pub u8,   hp_modem2active_backup_clk_sel       , set_hp_modem2active_backup_clk_sel       : 17, 16;
    pub u8,   hp_sleep2active_backup_mode          , set_hp_sleep2active_backup_mode          : 22, 18;
    pub u8,   hp_modem2active_backup_mode          , set_hp_modem2active_backup_mode          : 27, 23;
    pub bool, hp_sleep2active_backup_en            , set_hp_sleep2active_backup_en            : 29;
    pub bool, hp_modem2active_backup_en            , set_hp_modem2active_backup_en            : 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/modem
    struct HpModemBackup(u32);

    pub u8,   hp_sleep2modem_backup_modem_clk_code, set_hp_sleep2modem_backup_modem_clk_code: 5, 4;
    pub u8,   hp_sleep2modem_backup_clk_sel       , set_hp_sleep2modem_backup_clk_sel       : 15, 14;
    pub u8,   hp_sleep2modem_backup_mode          , set_hp_sleep2modem_backup_mode          : 24, 20;
    pub bool, hp_sleep2modem_backup_en            , set_hp_sleep2modem_backup_en            : 29;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_hp_backup_reg_t/sleep
    struct HpSleepBackup(u32);

    pub u8,   hp_modem2sleep_backup_modem_clk_code , set_hp_modem2sleep_backup_modem_clk_code : 7, 6;
    pub u8,   hp_active2sleep_backup_modem_clk_code, set_hp_active2sleep_backup_modem_clk_code: 9, 8;
    pub u8,   hp_modem2sleep_backup_clk_sel        , set_hp_modem2sleep_backup_clk_sel        : 17, 16;
    pub u8,   hp_active2sleep_backup_clk_sel       , set_hp_active2sleep_backup_clk_sel       : 19, 18;
    pub u8,   hp_modem2sleep_backup_mode           , set_hp_modem2sleep_backup_mode           : 24, 20;
    pub u8,   hp_active2sleep_backup_mode          , set_hp_active2sleep_backup_mode          : 29, 25;
    pub bool, hp_modem2sleep_backup_en             , set_hp_modem2sleep_backup_en             : 30;
    pub bool, hp_active2sleep_backup_en            , set_hp_active2sleep_backup_en            : 31;
}

/// Writes the parts of `pmu_hp_system_init` that are the same in every HP mode.
macro_rules! hp_system_init {
    ($state:ident => $s:ident) => {
        paste::paste! {
            unsafe {
                PMU::regs().[<$state _dig_power >]().write(|w| w.bits($s.power.dig_power.0));
                PMU::regs().[<$state _xtal >]().modify(|_, w| w
                    .[<$state _xpd_xtal >]().bit($s.power.xtal.xpd_xtal())
                );

                PMU::regs().[<$state _icg_hp_func >]().write(|w| w.bits($s.clock.icg_func));
                PMU::regs().[<$state _icg_hp_apb >]().write(|w| w.bits($s.clock.icg_apb));
                PMU::regs().[<$state _icg_modem >]().write(|w| w
                    .[<$state _dig_icg_modem_code >]().bits($s.clock.icg_modem_code)
                );
                PMU::regs().[<$state _sysclk >]().modify(|_, w| w
                    .[<$state _dig_sys_clk_no_div >]().bit($s.clock.sysclk.dig_sysclk_nodiv())
                    .[<$state _icg_sys_clock_en >]().bit($s.clock.sysclk.icg_sysclk_en())
                    .[<$state _sys_clk_slp_sel >]().bit($s.clock.sysclk.sysclk_slp_sel())
                    .[<$state _icg_slp_sel >]().bit($s.clock.sysclk.icg_slp_sel())
                    .[<$state _dig_sys_clk_sel >]().bits($s.clock.sysclk.dig_sysclk_sel())
                );

                PMU::regs().[<$state _hp_sys_cntl >]().modify(|_, w| w
                    .[<$state _uart_wakeup_en >]().bit($s.syscntl.uart_wakeup_en())
                    .[<$state _lp_pad_hold_all >]().bit($s.syscntl.lp_pad_hold_all())
                    .[<$state _hp_pad_hold_all >]().bit($s.syscntl.hp_pad_hold_all())
                    .[<$state _dig_pad_slp_sel >]().bit($s.syscntl.dig_pad_slp_sel())
                    .[<$state _dig_pause_wdt >]().bit($s.syscntl.dig_pause_wdt())
                    .[<$state _dig_cpu_stall >]().bit($s.syscntl.dig_cpu_stall())
                );

                PMU::regs().[<$state _bias >]().modify(|_, w| w
                    .[<$state _dcdc_ccm_enb >]().bit($s.anlg.bias.dcdc_ccm_enb())
                    .[<$state _dcdc_clear_rdy >]().bit($s.anlg.bias.dcdc_clear_rdy())
                    .[<$state _dig_pmu_dpcur_bias >]().bits($s.anlg.bias.dig_reg_dpcur_bias())
                    .[<$state _dig_pmu_dsfmos >]().bits($s.anlg.bias.dig_reg_dsfmos())
                    .[<$state _dcm_vset >]().bits($s.anlg.bias.dcm_vset())
                    .[<$state _dcm_mode >]().bits($s.anlg.bias.dcm_mode())
                    .[<$state _xpd_bias >]().bit($s.anlg.bias.xpd_bias())
                    .[<$state _xpd_trx >]().bit($s.anlg.bias.xpd_trx())
                    .[<$state _discnnt_dig_rtc >]().bit($s.anlg.bias.discnnt_dig_rtc())
                    .[<$state _pd_cur >]().bit($s.anlg.bias.pd_cur())
                    .sleep().bit($s.anlg.bias.bias_sleep())
                );

                PMU::regs().[<$state _hp_regulator0 >]().modify(|_, w| w
                    .[<$state _hp_power_det_bypass >]().bit($s.anlg.regulator0.power_det_bypass())
                    .[<$state _hp_regulator_slp_mem_xpd >]().bit($s.anlg.regulator0.slp_mem_xpd())
                    .[<$state _hp_regulator_slp_logic_xpd >]().bit($s.anlg.regulator0.slp_logic_xpd())
                    .[<$state _hp_regulator_xpd >]().bit($s.anlg.regulator0.xpd())
                    .[<$state _hp_regulator_slp_mem_dbias >]().bits($s.anlg.regulator0.slp_mem_dbias())
                    .[<$state _hp_regulator_slp_logic_dbias >]().bits($s.anlg.regulator0.slp_logic_dbias())
                    .[<$state _hp_regulator_dbias >]().bits($s.anlg.regulator0.dbias())
                );

                PMU::regs().[<$state _hp_regulator1 >]().modify(|_, w| w
                    .[<$state _hp_regulator_drv_b >]().bits($s.anlg.regulator1.drv_b())
                );

                PMU::regs().[<$state _backup >]().write(|w| w.bits($s.retention));
                PMU::regs().[<$state _backup_clk >]().write(|w| w.bits($s.backup_clk));
            }
        }
    };
}

// pmu_hp_system_param_t
struct HpSystemInit {
    power: HpSysPower,
    clock: SystemClockParam,
    syscntl: HpSysCntlReg,
    anlg: HpAnalog,
    retention: u32,
    backup_clk: u32,
}

impl HpSystemInit {
    /// The active-mode analog configuration, which the modem mode shares.
    ///
    /// PMU_HP_ACTIVE_ANALOG_CONFIG_DEFAULT, PMU_HP_MODEM_ANALOG_CONFIG_DEFAULT
    fn active_analog() -> HpAnalog {
        let mut anlg = HpAnalog::default();
        anlg.bias.set_dcdc_ccm_enb(false);
        anlg.bias.set_dcdc_clear_rdy(false);
        anlg.bias.set_dig_reg_dpcur_bias(3);
        anlg.bias.set_dig_reg_dsfmos(15);
        anlg.bias.set_dcm_mode(3);
        anlg.bias.set_dcm_vset(24);
        anlg.bias.set_xpd_trx(true);
        anlg.bias.set_xpd_bias(true);
        anlg.bias.set_discnnt_dig_rtc(false);
        anlg.bias.set_pd_cur(false);
        anlg.bias.set_bias_sleep(false);

        anlg.regulator0.set_power_det_bypass(false);
        anlg.regulator0.set_dbias_sel(true);
        anlg.regulator0.set_dbias_init(true);
        anlg.regulator0.set_slp_mem_xpd(false);
        anlg.regulator0.set_slp_logic_xpd(false);
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_slp_mem_dbias(0);
        anlg.regulator0.set_slp_logic_dbias(0);
        anlg.regulator0.set_dbias(0);

        // get_act_hp_drvb
        anlg.regulator1.set_drv_b(HP_CALI_DRVB);

        anlg
    }

    fn active() -> Self {
        // PMU_HP_ACTIVE_POWER_CONFIG_DEFAULT
        let mut power = HpSysPower::default();
        power.clk.set_i2c_iso_en(false);
        power.clk.set_i2c_retention(false);
        power.clk.set_xpd_bb_i2c(true);
        power.xtal.set_xpd_xtal(true);

        // PMU_HP_ACTIVE_CLOCK_CONFIG_DEFAULT
        let mut clock = SystemClockParam {
            icg_func: 0xffff_ffff,
            icg_apb: 0xffff_ffff,
            icg_modem_code: ICG_MODEM_CODE_ACTIVE,
            ..Default::default()
        };
        clock.sysclk.set_icg_sysclk_en(true);
        clock.sysclk.set_dig_sysclk_sel(PMU_CLK_SRC_XTAL);

        // PMU_HP_ACTIVE_DIGITAL_CONFIG_DEFAULT
        let syscntl = HpSysCntlReg::default();

        // PMU_HP_ACTIVE_RETENTION_CONFIG_DEFAULT
        let mut retention = HpActiveBackup::default();
        retention.set_hp_sleep2active_backup_modem_clk_code(2);
        retention.set_hp_modem2active_backup_modem_clk_code(2);
        retention.set_hp_sleep2active_backup_clk_sel(PMU_CLK_SRC_XTAL_X2);
        retention.set_hp_modem2active_backup_clk_sel(PMU_CLK_SRC_XTAL);
        retention.set_hp_sleep2active_backup_mode(hp_retention_regdma_config(0, 0));
        retention.set_hp_modem2active_backup_mode(hp_retention_regdma_config(0, 2));
        retention.set_hp_sleep2active_backup_en(false);
        retention.set_hp_modem2active_backup_en(false);

        Self {
            power,
            clock,
            syscntl,
            anlg: Self::active_analog(),
            retention: retention.0,
            backup_clk: 0xffff_ffff,
        }
    }

    fn modem() -> Self {
        // PMU_HP_MODEM_POWER_CONFIG_DEFAULT
        let mut power = HpSysPower::default();
        power.dig_power.set_cpu_pd_en(true);
        power.clk.set_xpd_bb_i2c(true);
        power.xtal.set_xpd_xtal(true);

        // PMU_HP_MODEM_CLOCK_CONFIG_DEFAULT
        let mut clock = SystemClockParam {
            icg_func: ICG_FUNC_ENA_HPBUS,
            icg_apb: 0,
            icg_modem_code: ICG_MODEM_CODE_MODEM,
            ..Default::default()
        };
        clock.sysclk.set_icg_sysclk_en(true);
        clock.sysclk.set_sysclk_slp_sel(true);
        clock.sysclk.set_icg_slp_sel(true);
        clock.sysclk.set_dig_sysclk_sel(PMU_CLK_SRC_PLL);

        // PMU_HP_MODEM_DIGITAL_CONFIG_DEFAULT
        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_uart_wakeup_en(true);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_dig_cpu_stall(true);

        // PMU_HP_MODEM_RETENTION_CONFIG_DEFAULT
        let mut retention = HpModemBackup::default();
        retention.set_hp_sleep2modem_backup_modem_clk_code(1);
        retention.set_hp_sleep2modem_backup_clk_sel(PMU_CLK_SRC_XTAL);
        retention.set_hp_sleep2modem_backup_mode(hp_retention_regdma_config(0, 1));
        retention.set_hp_sleep2modem_backup_en(false);

        Self {
            power,
            clock,
            syscntl,
            anlg: Self::active_analog(),
            retention: retention.0,
            backup_clk: 0xffff_ffff,
        }
    }

    fn sleep() -> Self {
        // PMU_HP_SLEEP_POWER_CONFIG_DEFAULT
        let mut power = HpSysPower::default();
        power.dig_power.set_vdd_flash_mode(3);
        power.clk.set_i2c_iso_en(true);
        power.clk.set_i2c_retention(true);

        // PMU_HP_SLEEP_CLOCK_CONFIG_DEFAULT
        let mut clock = SystemClockParam {
            icg_func: 0,
            icg_apb: 0,
            icg_modem_code: ICG_MODEM_CODE_SLEEP,
            ..Default::default()
        };
        clock.sysclk.set_sysclk_slp_sel(true);
        clock.sysclk.set_icg_slp_sel(true);
        clock.sysclk.set_dig_sysclk_sel(PMU_CLK_SRC_XTAL);

        // PMU_HP_SLEEP_DIGITAL_CONFIG_DEFAULT
        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_uart_wakeup_en(true);
        syscntl.set_dig_pad_slp_sel(true);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_dig_cpu_stall(true);

        // PMU_HP_SLEEP_ANALOG_CONFIG_DEFAULT
        let mut anlg = HpAnalog::default();
        anlg.bias.set_dcdc_ccm_enb(true);
        anlg.bias.set_dcdc_clear_rdy(false);
        anlg.bias.set_dig_reg_dpcur_bias(1);
        anlg.bias.set_dig_reg_dsfmos(4);
        anlg.bias.set_dcm_mode(3);
        anlg.bias.set_dcm_vset(20);
        anlg.bias.set_xpd_trx(false);
        anlg.bias.set_xpd_bias(false);
        anlg.bias.set_discnnt_dig_rtc(false);
        anlg.bias.set_pd_cur(true);
        anlg.bias.set_bias_sleep(true);
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_dbias(0);
        anlg.regulator1.set_drv_b(25);

        // PMU_HP_SLEEP_RETENTION_CONFIG_DEFAULT
        let mut retention = HpSleepBackup::default();
        retention.set_hp_modem2sleep_backup_modem_clk_code(0);
        retention.set_hp_active2sleep_backup_modem_clk_code(2);
        retention.set_hp_modem2sleep_backup_clk_sel(PMU_CLK_SRC_XTAL);
        retention.set_hp_active2sleep_backup_clk_sel(PMU_CLK_SRC_XTAL_X2);
        retention.set_hp_modem2sleep_backup_mode(hp_retention_regdma_config(1, 1));
        retention.set_hp_active2sleep_backup_mode(hp_retention_regdma_config(1, 0));
        retention.set_hp_modem2sleep_backup_en(false);
        retention.set_hp_active2sleep_backup_en(false);

        Self {
            power,
            clock,
            syscntl,
            anlg,
            retention: retention.0,
            backup_clk: 0xffff_ffff,
        }
    }

    fn init_default() {
        let active = Self::active();
        let modem = Self::modem();
        let sleep = Self::sleep();

        hp_system_init!(hp_active => active);
        hp_system_init!(hp_modem => modem);
        hp_system_init!(hp_sleep => sleep);

        // The clock tree owns the power of the root clock circuits (BBPLL, XTAL_X2) in the active
        // mode, so only the analog I2C master is configured there.
        PMU::regs().hp_active_hp_ck_power().modify(|_, w| {
            w.hp_active_xpd_bb_i2c().bit(active.power.clk.xpd_bb_i2c());
            w.hp_active_i2c_iso_en().bit(active.power.clk.i2c_iso_en());
            w.hp_active_i2c_retention()
                .bit(active.power.clk.i2c_retention())
        });
        PMU::regs()
            .hp_modem_hp_ck_power()
            .write(|w| unsafe { w.bits(modem.power.clk.0) });
        PMU::regs()
            .hp_modem_xtal()
            .modify(|_, w| w.hp_modem_xpd_xtalx2().bit(modem.power.xtal.xpd_xtalx2()));
        PMU::regs()
            .hp_sleep_hp_ck_power()
            .write(|w| unsafe { w.bits(sleep.power.clk.0) });
        PMU::regs()
            .hp_sleep_xtal()
            .modify(|_, w| w.hp_sleep_xpd_xtalx2().bit(sleep.power.xtal.xpd_xtalx2()));

        // Only the HP_ACTIVE mode has the regulator select bits.
        PMU::regs().hp_active_hp_regulator0().modify(|_, w| {
            w.dig_regulator0_dbias_sel()
                .bit(active.anlg.regulator0.dbias_sel());
            w.dig_dbias_init().bit(active.anlg.regulator0.dbias_init())
        });

        // Some PMU initial parameter configuration
        PMU::regs()
            .imm_modem_icg()
            .write(|w| w.update_dig_icg_modem_en().set_bit());
        PMU::regs()
            .imm_sleep_sysclk()
            .write(|w| w.update_dig_icg_switch().set_bit());

        const PMU_SLEEP_PROTECT_HP_LP_SLEEP: u8 = 2;
        PMU::regs()
            .slp_wakeup_cntl3()
            .modify(|_, w| unsafe { w.sleep_prt_sel().bits(PMU_SLEEP_PROTECT_HP_LP_SLEEP) });

        // pmu_ll_set_dcdc_ccm_sw_en
        PMU::regs()
            .dcm_ctrl()
            .modify(|_, w| w.dcdc_ccm_sw_en().set_bit());

        // lp_aon_ll_set_ldo_sw: the DC-DC LDO mode for a VDD below about 2.6 V.
        const DREG_LDO_HW: u32 = 1 << 5;
        const DREG_LDO_SW_SHIFT: u32 = 11;
        const DREG_LDO_SW_MASK: u32 = 0xf << DREG_LDO_SW_SHIFT;
        LP_AON::regs().date().modify(|r, w| unsafe {
            w.bits((r.bits() & !(DREG_LDO_HW | DREG_LDO_SW_MASK)) | (15 << DREG_LDO_SW_SHIFT))
        });

        // The BLE bandgap uses the calibrated ocode.
        let ulp_ocode = if regi2c::I2C_ULP_IR_FORCE_CODE.read() != 0 {
            regi2c::I2C_ULP_EXT_CODE.read()
        } else {
            regi2c::I2C_ULP_OCODE.read()
        };
        PMU::regs().ble_bandgap_ctrl().modify(|_, w| unsafe {
            w.ext_ocode().bits(ulp_ocode);
            w.ext_force_ocode().set_bit()
        });
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.0
    pub struct LpDigPower(u32);

    pub u8,   vdd_io_mode   , set_vdd_io_mode   : 26, 23;
    pub bool, bod_source_sel, set_bod_source_sel: 27;
    pub u8,   vddbat_mode   , set_vddbat_mode   : 29, 28;
    pub bool, mem_dslp      , set_mem_dslp      : 30;
    pub bool, peri_pd_en    , set_peri_pd_en    : 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    // pmu_lp_power_t.1
    pub struct LpClkPower(u32);

    pub bool, xpd_lppll  , set_xpd_lppll  : 27;
    pub bool, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub bool, xpd_rc32k  , set_xpd_rc32k  : 29;
    pub bool, xpd_fosc   , set_xpd_fosc   : 30;
    pub bool, pd_osc     , set_pd_osc     : 31;
}

#[derive(Clone, Copy, Default)]
// pmu_sleep_power_config_t.lp_sys
pub struct LpSysPower {
    pub dig_power: LpDigPower,
    pub clk_power: LpClkPower,
    // Only the LP_SLEEP mode has this register.
    pub xtal: XtalPower,
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

    pub u8, drv_b, set_drv_b: 31, 28;
}

#[derive(Clone, Copy, Default)]
// pmu_lp_analog_t
pub struct LpAnalog {
    // Only the LP_SLEEP mode has this register.
    pub bias: AnalogBias,
    pub regulator0: LpAnalogRegulator0,
    pub regulator1: LpAnalogRegulator1,
}

/// Writes the parts of `pmu_lp_system_init` that are the same in every LP mode.
macro_rules! lp_system_init {
    ($state:ident => $s:ident) => {
        paste::paste! {
            unsafe {
                PMU::regs().[< $state _dig_power >]().write(|w| w.bits($s.power.dig_power.0));
                PMU::regs().[< $state _ck_power >]().write(|w| w.bits($s.power.clk_power.0));

                PMU::regs().[< $state _regulator0 >]().modify(|_, w| w
                    .[< $state _regulator_slp_xpd >]().bit($s.anlg.regulator0.slp_xpd())
                    .[< $state _regulator_xpd >]().bit($s.anlg.regulator0.xpd())
                    .[< $state _regulator_slp_dbias >]().bits($s.anlg.regulator0.slp_dbias())
                    .[< $state _regulator_dbias >]().bits($s.anlg.regulator0.dbias())
                );

                PMU::regs().[< $state _regulator1 >]().modify(|_, w| w
                    .[< $state _regulator_drv_b >]().bits($s.anlg.regulator1.drv_b())
                );
            }
        }
    };
}

// pmu_lp_system_param_t
struct LpSystemInit {
    power: LpSysPower,
    anlg: LpAnalog,
}

impl LpSystemInit {
    fn active() -> Self {
        // PMU_LP_ACTIVE_POWER_CONFIG_DEFAULT
        let mut power = LpSysPower::default();
        power.clk_power.set_xpd_xtal32k(xtal32k::use_xtal32k());
        power.clk_power.set_xpd_fosc(true);

        // PMU_LP_ACTIVE_ANALOG_CONFIG_DEFAULT
        let mut anlg = LpAnalog::default();
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_dbias(LP_CALI_DBIAS);

        Self { power, anlg }
    }

    fn sleep() -> Self {
        // PMU_LP_SLEEP_POWER_CONFIG_DEFAULT
        let power = LpSysPower::default();

        // PMU_LP_SLEEP_ANALOG_CONFIG_DEFAULT
        let mut anlg = LpAnalog::default();
        anlg.bias.set_dcdc_ccm_enb(true);
        anlg.bias.set_dcdc_clear_rdy(false);
        anlg.bias.set_dig_reg_dpcur_bias(1);
        anlg.bias.set_dig_reg_dsfmos(4);
        anlg.bias.set_dcm_mode(3);
        anlg.bias.set_dcm_vset(20);
        anlg.bias.set_xpd_bias(false);
        anlg.bias.set_discnnt_dig_rtc(true);
        anlg.bias.set_pd_cur(true);
        anlg.bias.set_bias_sleep(true);
        anlg.regulator0.set_xpd(true);
        anlg.regulator0.set_dbias(3);

        Self { power, anlg }
    }

    fn init_default() {
        let active = Self::active();
        let sleep = Self::sleep();

        // The LP_ACTIVE registers are the ones named after HP_SLEEP.
        lp_system_init!(hp_sleep_lp => active);
        lp_system_init!(lp_sleep_lp => sleep);

        PMU::regs().lp_sleep_xtal().modify(|_, w| {
            w.lp_sleep_xpd_xtal().bit(sleep.power.xtal.xpd_xtal());
            w.lp_sleep_xpd_xtalx2().bit(sleep.power.xtal.xpd_xtalx2())
        });

        let bias = sleep.anlg.bias;
        PMU::regs().lp_sleep_bias().modify(|_, w| unsafe {
            w.lp_sleep_dcdc_ccm_enb().bit(bias.dcdc_ccm_enb());
            w.lp_sleep_dcdc_clear_rdy().bit(bias.dcdc_clear_rdy());
            w.lp_sleep_dig_pmu_dpcur_bias()
                .bits(bias.dig_reg_dpcur_bias());
            w.lp_sleep_dig_pmu_dsfmos().bits(bias.dig_reg_dsfmos());
            w.lp_sleep_dcm_vset().bits(bias.dcm_vset());
            w.lp_sleep_dcm_mode().bits(bias.dcm_mode());
            w.lp_sleep_xpd_bias().bit(bias.xpd_bias());
            w.lp_sleep_discnnt_dig_rtc().bit(bias.discnnt_dig_rtc());
            w.lp_sleep_pd_cur().bit(bias.pd_cur());
            w.sleep().bit(bias.bias_sleep())
        });
    }
}

pub(crate) fn init(_config: &ClockConfig) {
    // pmu_init()

    // Peripheral reg i2c power up
    PMU::regs().rf_pwc().modify(|_, w| {
        w.xpd_perif_i2c().set_bit();
        w.xpd_rftx_i2c().set_bit();
        w.xpd_rfrx_i2c().set_bit()
    });

    HpSystemInit::init_default();
    LpSystemInit::init_default();

    pmu_power_domain_force_default();

    // This is the part of esp-idf's `rtc_clk_init` that has to run before the CPU can be switched
    // to the BBPLL.

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

    // Power down the RF PLL.
    PMU::regs()
        .date()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 25)) });

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

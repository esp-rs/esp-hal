use strum::FromRepr;

use crate::{
    peripherals::{MODEM_LPCON, MODEM_SYSCON, PMU},
    soc::{
        clocks::{ClockConfig, LpSlowClkConfig},
        regi2c,
    },
};

const HP_CALI_DBIAS: u32 = 26;
const LP_CALI_DBIAS: u32 = 26;

const ICG_NOGATING_MODEM: u8 = 1 << 1;
const ICG_NOGATING_ACTIVE: u8 = 1 << 2;
const ICG_NOGATING_ACTIVE_MODEM: u8 = ICG_NOGATING_ACTIVE | ICG_NOGATING_MODEM;

const fn hp_retention_regdma_config(dir: u32, entry: u32) -> u32 {
    ((dir << 4) | (entry & 0xf)) & 0x1f
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpDigPower(u32);

    pub bool, vdd_spi_pd_en, set_vdd_spi_pd_en: 20;
    pub bool, hp_alive_pd_en, set_hp_alive_pd_en: 21;
    pub bool, hp_mem_dslp, set_hp_mem_dslp: 22;
    pub u8, hp_mem_pd_en, set_hp_mem_pd_en: 26, 23;
    pub bool, modem_top_pd_en, set_modem_top_pd_en: 27;
    pub bool, hp_cnnt_pd_en, set_hp_cnnt_pd_en: 28;
    pub bool, hp_cpu_pd_en, set_hp_cpu_pd_en: 29;
    pub bool, modem_pwr_pd_en, set_modem_pwr_pd_en: 30;
    pub bool, top_pd_en, set_top_pd_en: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpClkPower(u32);

    pub bool, xpd_xtalx2, set_xpd_xtalx2: 19;
    pub bool, i2c_iso_en, set_i2c_iso_en: 20;
    pub bool, i2c_retention, set_i2c_retention: 21;
    pub bool, xpd_bb_i2c, set_xpd_bb_i2c: 22;
    pub bool, xpd_cpll_i2c, set_xpd_cpll_i2c: 23;
    pub bool, xpd_bbpll_i2c, set_xpd_bbpll_i2c: 24;
    pub bool, xpd_apll_i2c, set_xpd_apll_i2c: 25;
    pub bool, xpd_mpll_i2c, set_xpd_mpll_i2c: 26;
    pub bool, xpd_cpll, set_xpd_cpll: 27;
    pub bool, xpd_bbpll, set_xpd_bbpll: 28;
    pub bool, xpd_apll, set_xpd_apll: 29;
    pub bool, xpd_mpll, set_xpd_mpll: 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct XtalPower(u32);

    pub bool, xpd_xtal, set_xpd_xtal: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpIcgModem(u32);

    pub u8, code, set_code: 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpSysclk(u32);

    pub bool, dig_sysclk_nodiv, set_dig_sysclk_nodiv: 26;
    pub bool, icg_sysclk_en, set_icg_sysclk_en: 27;
    pub bool, sysclk_slp_sel, set_sysclk_slp_sel: 28;
    pub bool, icg_slp_sel, set_icg_slp_sel: 29;
    pub u8, dig_sysclk_sel, set_dig_sysclk_sel: 31, 30;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpSysCntl(u32);

    pub bool, c_channel, set_c_channel: 23;
    pub bool, uart_wakeup_en, set_uart_wakeup_en: 24;
    pub bool, lp_pad_hold_all, set_lp_pad_hold_all: 25;
    pub bool, hp_pad_hold_all, set_hp_pad_hold_all: 26;
    pub bool, dig_pad_slp_sel, set_dig_pad_slp_sel: 27;
    pub bool, dig_pause_wdt, set_dig_pause_wdt: 28;
    pub bool, dig_cpu_stall, set_dig_cpu_stall: 29;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct AnalogBias(u32);

    pub bool, xpd_bias, set_xpd_bias: 25;
    pub u8, dbg_atten, set_dbg_atten: 29, 26;
    pub bool, pd_cur, set_pd_cur: 30;
    pub bool, bias_sleep, set_bias_sleep: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpRegulator0(u32);

    pub bool, dig_dbias_init, set_dig_dbias_init: 3;
    pub u8, lp_dbias_vol, set_lp_dbias_vol: 8, 4;
    pub u8, hp_dbias_vol, set_hp_dbias_vol: 13, 9;
    pub bool, dbias_sel, set_dbias_sel: 14;
    pub bool, slp_connect_en, set_slp_connect_en: 15;
    pub bool, slp_mem_xpd, set_slp_mem_xpd: 16;
    pub bool, slp_logic_xpd, set_slp_logic_xpd: 17;
    pub bool, xpd, set_xpd: 18;
    pub u8, slp_mem_dbias, set_slp_mem_dbias: 22, 19;
    pub u8, slp_logic_dbias, set_slp_logic_dbias: 26, 23;
    pub u8, dbias, set_dbias: 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpRegulator1(u32);

    pub u32, drv_b, set_drv_b: 31, 8;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct HpBackup(u32);

    pub u8, sleep2active_modem_clk_code, set_sleep2active_modem_clk_code: 5, 4;
    pub u8, modem2active_modem_clk_code, set_modem2active_modem_clk_code: 7, 6;
    pub bool, active_retention_mode, set_active_retention_mode: 10;
    pub bool, sleep2active_retention_en, set_sleep2active_retention_en: 11;
    pub bool, modem2active_retention_en, set_modem2active_retention_en: 12;
    pub u8, sleep2active_backup_clk_sel, set_sleep2active_backup_clk_sel: 15, 14;
    pub u8, modem2active_backup_clk_sel, set_modem2active_backup_clk_sel: 17, 16;
    pub u8, sleep2active_backup_mode, set_sleep2active_backup_mode: 22, 18;
    pub u8, modem2active_backup_mode, set_modem2active_backup_mode: 27, 23;
    pub bool, sleep2active_backup_en, set_sleep2active_backup_en: 29;
    pub bool, modem2active_backup_en, set_modem2active_backup_en: 30;

    pub u8, sleep2modem_modem_clk_code, set_sleep2modem_modem_clk_code: 5, 4;
    pub bool, modem_retention_mode, set_modem_retention_mode: 10;
    pub bool, sleep2modem_retention_en, set_sleep2modem_retention_en: 11;
    pub u8, sleep2modem_backup_clk_sel, set_sleep2modem_backup_clk_sel: 15, 14;
    pub u8, sleep2modem_backup_mode, set_sleep2modem_backup_mode: 24, 20;
    pub bool, sleep2modem_backup_en, set_sleep2modem_backup_en: 29;

    pub u8, modem2sleep_modem_clk_code, set_modem2sleep_modem_clk_code: 7, 6;
    pub u8, active2sleep_modem_clk_code, set_active2sleep_modem_clk_code: 9, 8;
    pub bool, sleep_retention_mode, set_sleep_retention_mode: 10;
    pub bool, modem2sleep_retention_en, set_modem2sleep_retention_en: 12;
    pub bool, active2sleep_retention_en, set_active2sleep_retention_en: 13;
    pub u8, modem2sleep_backup_clk_sel, set_modem2sleep_backup_clk_sel: 17, 16;
    pub u8, active2sleep_backup_clk_sel, set_active2sleep_backup_clk_sel: 19, 18;
    pub u8, modem2sleep_backup_mode, set_modem2sleep_backup_mode: 24, 20;
    pub u8, active2sleep_backup_mode, set_active2sleep_backup_mode: 29, 25;
    pub bool, modem2sleep_backup_en, set_modem2sleep_backup_en: 30;
    pub bool, active2sleep_backup_en, set_active2sleep_backup_en: 31;
}

#[derive(Clone, Copy, Default)]
struct HpSystemPower {
    dig_power: HpDigPower,
    clk_power: HpClkPower,
    xtal: XtalPower,
}

#[derive(Clone, Copy)]
struct HpSystemClock {
    icg_func: [u32; 2],
    icg_apb: [u32; 2],
    icg_modem: HpIcgModem,
    sysclk: HpSysclk,
}

#[derive(Clone, Copy)]
struct HpSystemDigital {
    syscntl: HpSysCntl,
}

#[derive(Clone, Copy)]
struct HpSystemAnalog {
    bias: AnalogBias,
    regulator0: HpRegulator0,
    regulator1: HpRegulator1,
}

#[derive(Clone, Copy)]
struct HpSystemRetention {
    retention: HpBackup,
    backup_clk: [u32; 2],
}

#[derive(Clone, Copy)]
struct HpSystemInit {
    power: HpSystemPower,
    clock: HpSystemClock,
    digital: HpSystemDigital,
    analog: HpSystemAnalog,
    retention: HpSystemRetention,
}

impl HpSystemInit {
    // PMU_HP_ACTIVE_*_CONFIG_DEFAULT
    fn active() -> Self {
        let mut power = HpSystemPower::default();
        power.clk_power.set_xpd_bb_i2c(true);
        power.clk_power.set_xpd_cpll_i2c(true);
        power.clk_power.set_xpd_bbpll_i2c(true);
        power.clk_power.set_xpd_mpll_i2c(true);
        power.clk_power.set_xpd_cpll(true);
        power.clk_power.set_xpd_bbpll(true);
        power.clk_power.set_xpd_mpll(true);
        power.xtal.set_xpd_xtal(true);

        let mut icg_modem = HpIcgModem::default();
        icg_modem.set_code(2);
        let mut sysclk = HpSysclk::default();
        sysclk.set_icg_sysclk_en(true);
        let mut syscntl = HpSysCntl::default();
        syscntl.set_c_channel(true);

        let mut bias = AnalogBias::default();
        bias.set_xpd_bias(true);
        let mut regulator0 = HpRegulator0::default();
        regulator0.set_dig_dbias_init(true);
        regulator0.set_lp_dbias_vol(0xd);
        regulator0.set_hp_dbias_vol(0x1c);
        regulator0.set_dbias_sel(true);
        regulator0.set_xpd(true);
        regulator0.set_dbias(HP_CALI_DBIAS as u8);

        let mut retention = HpBackup::default();
        retention.set_sleep2active_modem_clk_code(2);
        retention.set_modem2active_modem_clk_code(2);
        retention.set_sleep2active_backup_mode(hp_retention_regdma_config(0, 0) as u8);
        retention.set_modem2active_backup_mode(hp_retention_regdma_config(0, 2) as u8);

        Self {
            power,
            clock: HpSystemClock {
                icg_func: [u32::MAX; 2],
                icg_apb: [u32::MAX; 2],
                icg_modem,
                sysclk,
            },
            digital: HpSystemDigital { syscntl },
            analog: HpSystemAnalog {
                bias,
                regulator0,
                regulator1: HpRegulator1::default(),
            },
            retention: HpSystemRetention {
                retention,
                backup_clk: [u32::MAX; 2],
            },
        }
    }

    // PMU_HP_MODEM_*_CONFIG_DEFAULT
    fn modem() -> Self {
        let mut power = HpSystemPower::default();
        power.clk_power.set_i2c_iso_en(true);
        power.clk_power.set_i2c_retention(true);
        power.clk_power.set_xpd_bb_i2c(true);
        power.clk_power.set_xpd_cpll_i2c(true);
        power.clk_power.set_xpd_bbpll_i2c(true);
        power.clk_power.set_xpd_cpll(true);
        power.clk_power.set_xpd_bbpll(true);
        power.xtal.set_xpd_xtal(true);

        let mut icg_modem = HpIcgModem::default();
        icg_modem.set_code(1);
        let mut sysclk = HpSysclk::default();
        sysclk.set_icg_sysclk_en(true);
        sysclk.set_sysclk_slp_sel(true);
        sysclk.set_icg_slp_sel(true);
        let mut syscntl = HpSysCntl::default();
        syscntl.set_c_channel(true);
        syscntl.set_lp_pad_hold_all(true);
        syscntl.set_hp_pad_hold_all(true);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_dig_cpu_stall(true);

        let mut regulator0 = HpRegulator0::default();
        regulator0.set_xpd(true);
        regulator0.set_dbias(HP_CALI_DBIAS as u8);

        let mut retention = HpBackup::default();
        retention.set_sleep2modem_modem_clk_code(1);
        retention.set_sleep2modem_backup_mode(hp_retention_regdma_config(0, 1) as u8);

        Self {
            power,
            clock: HpSystemClock {
                // PMU_ICG_FUNC_ENA_ETM=40 and PMU_ICG_FUNC_ENA_BUS=47.
                icg_func: [0, (1 << (40 - 32)) | (1 << (47 - 32))],
                icg_apb: [0; 2],
                icg_modem,
                // The S31 IDF deliberately keeps the modem-state system clock on XTAL.
                sysclk,
            },
            digital: HpSystemDigital { syscntl },
            analog: HpSystemAnalog {
                bias: AnalogBias::default(),
                regulator0,
                regulator1: HpRegulator1::default(),
            },
            retention: HpSystemRetention {
                retention,
                backup_clk: [u32::MAX; 2],
            },
        }
    }

    // PMU_HP_SLEEP_*_CONFIG_DEFAULT
    fn sleep() -> Self {
        let mut power = HpSystemPower::default();
        power.clk_power.set_i2c_iso_en(true);
        power.clk_power.set_i2c_retention(true);

        let mut sysclk = HpSysclk::default();
        sysclk.set_sysclk_slp_sel(true);
        sysclk.set_icg_slp_sel(true);
        let mut syscntl = HpSysCntl::default();
        syscntl.set_uart_wakeup_en(true);
        syscntl.set_lp_pad_hold_all(true);
        syscntl.set_hp_pad_hold_all(true);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_dig_cpu_stall(true);

        let mut bias = AnalogBias::default();
        bias.set_pd_cur(true);
        bias.set_bias_sleep(true);
        let mut regulator0 = HpRegulator0::default();
        regulator0.set_xpd(true);

        let mut retention = HpBackup::default();
        retention.set_active2sleep_modem_clk_code(2);
        retention.set_modem2sleep_backup_mode(hp_retention_regdma_config(1, 1) as u8);
        retention.set_active2sleep_backup_mode(hp_retention_regdma_config(1, 0) as u8);

        Self {
            power,
            clock: HpSystemClock {
                icg_func: [0; 2],
                icg_apb: [0; 2],
                icg_modem: HpIcgModem::default(),
                sysclk,
            },
            digital: HpSystemDigital { syscntl },
            analog: HpSystemAnalog {
                bias,
                regulator0,
                regulator1: HpRegulator1::default(),
            },
            retention: HpSystemRetention {
                retention,
                backup_clk: [u32::MAX; 2],
            },
        }
    }
}

macro_rules! hp_system_init {
    ($state:ident, $param:expr) => {{
        let param = $param;
        let pmu = PMU::regs();
        paste::paste! {
            unsafe {
                pmu.[<$state _dig_power>]().write(|w| w.bits(param.power.dig_power.0));
                pmu.[<$state _hp_ck_power>]().write(|w| w.bits(param.power.clk_power.0));
                pmu.[<$state _xtal>]().write(|w| w.bits(param.power.xtal.0));

                pmu.[<$state _icg_hp_func>]().write(|w| w.bits(param.clock.icg_func[0]));
                pmu.[<$state _icg_hp_apb>]().write(|w| w.bits(param.clock.icg_apb[0]));
                pmu.[<$state _icg_hp_func1>]().write(|w| w.bits(param.clock.icg_func[1]));
                pmu.[<$state _icg_hp_apb1>]().write(|w| w.bits(param.clock.icg_apb[1]));
                pmu.[<$state _icg_modem>]().write(|w| w.bits(param.clock.icg_modem.0));
                pmu.[<$state _sysclk>]().write(|w| w.bits(param.clock.sysclk.0));

                pmu.[<$state _hp_sys_cntl>]().write(|w| w.bits(param.digital.syscntl.0));

                pmu.[<$state _bias>]().write(|w| w.bits(param.analog.bias.0));
                pmu.[<$state _hp_regulator0>]().write(|w| w.bits(param.analog.regulator0.0));
                pmu.[<$state _hp_regulator1>]().write(|w| w.bits(param.analog.regulator1.0));

                pmu.[<$state _backup>]().write(|w| w.bits(param.retention.retention.0));
                pmu.[<$state _backup_clk>]().write(|w| w.bits(param.retention.backup_clk[0]));
                pmu.[<$state _backup1_clk>]().write(|w| w.bits(param.retention.backup_clk[1]));
            }
        }
    }};
}

fn update_hp_system_config() {
    PMU::regs()
        .imm_modem_icg()
        .write(|w| w.update_dig_icg_modem_en().set_bit());
    PMU::regs()
        .imm_sleep_sysclk()
        .write(|w| w.update_dig_icg_switch().set_bit());
}

impl HpSystemInit {
    fn init_default() {
        hp_system_init!(hp_active, Self::active());
        update_hp_system_config();
        hp_system_init!(hp_modem, Self::modem());
        update_hp_system_config();
        hp_system_init!(hp_sleep, Self::sleep());
        update_hp_system_config();
        PMU::regs()
            .slp_wakeup_cntl3()
            .modify(|_, w| unsafe { w.sleep_prt_sel().bits(2) });
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct LpDigPower(u32);

    pub bool, mem_dslp, set_mem_dslp: 30;
    pub bool, peri_pd_en, set_peri_pd_en: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct LpClkPower(u32);

    pub bool, xpd_xtal32k, set_xpd_xtal32k: 28;
    pub bool, xpd_rc32k, set_xpd_rc32k: 29;
    pub bool, xpd_fosc, set_xpd_fosc: 30;
    pub bool, pd_osc, set_pd_osc: 31;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct LpRegulator0(u32);

    pub bool, slp_xpd, set_slp_xpd: 21;
    pub bool, xpd, set_xpd: 22;
    pub u8, slp_dbias, set_slp_dbias: 26, 23;
    pub u8, dbias, set_dbias: 31, 27;
}

bitfield::bitfield! {
    #[derive(Clone, Copy, Default)]
    pub struct LpRegulator1(u32);

    pub u8, drv_b, set_drv_b: 31, 28;
}

#[derive(Clone, Copy, Default)]
struct LpSystemPower {
    dig_power: LpDigPower,
    clk_power: LpClkPower,
    xtal: XtalPower,
}

#[derive(Clone, Copy, Default)]
struct LpSystemAnalog {
    bias: AnalogBias,
    regulator0: LpRegulator0,
    regulator1: LpRegulator1,
}

#[derive(Clone, Copy)]
struct LpSystemInit {
    power: LpSystemPower,
    analog: LpSystemAnalog,
}

impl LpSystemInit {
    // PMU_LP_ACTIVE_*_CONFIG_DEFAULT
    fn active() -> Self {
        let mut clk_power = LpClkPower::default();
        clk_power.set_xpd_fosc(true);
        let mut regulator0 = LpRegulator0::default();
        regulator0.set_xpd(true);
        regulator0.set_dbias(LP_CALI_DBIAS as u8);

        Self {
            power: LpSystemPower {
                dig_power: LpDigPower::default(),
                clk_power,
                xtal: XtalPower::default(),
            },
            analog: LpSystemAnalog {
                bias: AnalogBias::default(),
                regulator0,
                regulator1: LpRegulator1::default(),
            },
        }
    }

    // PMU_LP_SLEEP_*_CONFIG_DEFAULT
    fn sleep() -> Self {
        let mut bias = AnalogBias::default();
        bias.set_pd_cur(true);
        bias.set_bias_sleep(true);
        let mut regulator0 = LpRegulator0::default();
        regulator0.set_xpd(true);

        Self {
            power: LpSystemPower::default(),
            analog: LpSystemAnalog {
                bias,
                regulator0,
                regulator1: LpRegulator1::default(),
            },
        }
    }
}

impl LpSystemInit {
    fn init_default() {
        let active = Self::active();
        let sleep = Self::sleep();
        let pmu = PMU::regs();

        unsafe {
            // PMU_MODE_LP_ACTIVE is represented by the HP-sleep LP register group.
            pmu.hp_sleep_lp_dig_power()
                .write(|w| w.bits(active.power.dig_power.0));
            pmu.hp_sleep_lp_ck_power()
                .write(|w| w.bits(active.power.clk_power.0));
            pmu.hp_sleep_lp_regulator0()
                .write(|w| w.bits(active.analog.regulator0.0));
            pmu.hp_sleep_lp_regulator1()
                .write(|w| w.bits(active.analog.regulator1.0));

            pmu.lp_sleep_lp_dig_power()
                .write(|w| w.bits(sleep.power.dig_power.0));
            pmu.lp_sleep_lp_ck_power()
                .write(|w| w.bits(sleep.power.clk_power.0));
            pmu.lp_sleep_bias().write(|w| w.bits(sleep.analog.bias.0));
            pmu.lp_sleep_lp_regulator0()
                .write(|w| w.bits(sleep.analog.regulator0.0));
            pmu.lp_sleep_lp_regulator1()
                .write(|w| w.bits(sleep.analog.regulator1.0));
            pmu.lp_sleep_xtal().write(|w| w.bits(sleep.power.xtal.0));
        }
    }
}

fn power_domain_force_default() {
    let pmu = PMU::regs();

    // Clear the six force-control bits for every S31 HP power domain while
    // preserving the domain mask fields.
    macro_rules! clear_force_control {
        ($reg:expr) => {
            $reg.modify(|r, w| unsafe { w.bits(r.bits() & !0x3f) })
        };
    }
    clear_force_control!(pmu.power_pd_top_cntl());
    clear_force_control!(pmu.power_pd_hp_alive_cntl());
    clear_force_control!(pmu.power_pd_modem_pwr_cntl());
    clear_force_control!(pmu.power_pd_hpcpu_cntl());
    clear_force_control!(pmu.power_pd_hpcnnt_cntl());
    clear_force_control!(pmu.power_pd_modem_top_cntl());

    // Isolate all HP memory banks in sleep and disable force-power-up.
    pmu.power_pd_mem_cntl()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0xff00_0000) });
    pmu.power_pd_lpperi_cntl()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x3f) });
}

fn modem_clock_domain_power_state_icg_map_init() {
    // OR in the defaults from esp32s31/modem_clock_impl.c. Other users may
    // already own additional state-map bits, so do not replace whole fields.
    MODEM_SYSCON::regs()
        .clk_conf_power_st()
        .modify(|r, w| unsafe {
            w.clk_modem_apb_st_map()
                .bits(r.clk_modem_apb_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_modem_peri_st_map()
                .bits(r.clk_modem_peri_st_map().bits() | ICG_NOGATING_ACTIVE);
            w.clk_wifi_st_map()
                .bits(r.clk_wifi_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_bt_st_map()
                .bits(r.clk_bt_st_map().bits() | ICG_NOGATING_ACTIVE);
            w.clk_fe_st_map()
                .bits(r.clk_fe_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_zb_st_map()
                .bits(r.clk_zb_st_map().bits() | ICG_NOGATING_ACTIVE)
        });
    MODEM_LPCON::regs()
        .clk_conf_power_st()
        .modify(|r, w| unsafe {
            w.clk_lp_apb_st_map()
                .bits(r.clk_lp_apb_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_i2c_mst_st_map()
                .bits(r.clk_i2c_mst_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_coex_st_map()
                .bits(r.clk_coex_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM);
            w.clk_wifipwr_st_map()
                .bits(r.clk_wifipwr_st_map().bits() | ICG_NOGATING_ACTIVE_MODEM)
        });
}

/// Select the WiFi LP clock after `ClockConfig::configure` has applied and
/// acquired the configured LP clock source.
pub(crate) fn configure_wifi_lp_clock(config: &ClockConfig) {
    let lpcon = MODEM_LPCON::regs();
    let source = config.lp_slow_clk.unwrap_or(LpSlowClkConfig::RcSlow);

    lpcon.wifi_lp_clk_conf().modify(|_, w| {
        w.clk_wifipwr_lp_sel_osc_slow().clear_bit();
        w.clk_wifipwr_lp_sel_osc_fast().clear_bit();
        w.clk_wifipwr_lp_sel_xtal().clear_bit();
        w.clk_wifipwr_lp_sel_xtal32k().clear_bit();
        match source {
            LpSlowClkConfig::RcSlow => w.clk_wifipwr_lp_sel_osc_slow().set_bit(),
            LpSlowClkConfig::Xtal32k => w.clk_wifipwr_lp_sel_xtal32k().set_bit(),
        }
    });
    if source == LpSlowClkConfig::Xtal32k {
        // The WiFi 32 kHz selector feeds from the shared modem 32 kHz mux.
        // MODEM_CLOCK_XTAL32K_CODE is 0 on S31.
        lpcon
            .modem_32k_clk_conf()
            .modify(|_, w| unsafe { w.clk_modem_32k_sel().bits(0) });
    }
    lpcon
        .wifi_lp_clk_conf()
        .modify(|_, w| unsafe { w.clk_wifipwr_lp_div_num().bits(0) });
    lpcon.clk_conf().modify(|_, w| w.clk_wifipwr_en().set_bit());
}

pub(crate) fn init(_config: &ClockConfig) {
    // pmu_init: power up and release reset for the analog peripheral I2C.
    PMU::regs()
        .ana_peri_pwr_ctrl()
        .modify(|_, w| w.rstb_perif_i2c().clear_bit());
    crate::rom::ets_delay_us(1);
    PMU::regs().ana_peri_pwr_ctrl().modify(|_, w| {
        w.xpd_perif_i2c().set_bit();
        w.rstb_perif_i2c().set_bit()
    });

    HpSystemInit::init_default();
    LpSystemInit::init_default();
    power_domain_force_default();

    // rtc_clk_init: S31 only applies RC_SLOW tuning and regulator force bits
    // here, it has no PCR/FOSC/RC32K tuning sequence.
    regi2c::I2C_DIG_REG_SCK_DCAP.write_field(128);
    regi2c::I2C_DIG_REG_ENIF_RTC_DREG.write_field(1);
    regi2c::I2C_DIG_REG_ENIF_DIG_DREG.write_field(1);
    regi2c::I2C_DIG_REG_XPD_RTC_REG.write_field(0);
    regi2c::I2C_DIG_REG_XPD_DIG_REG.write_field(0);

    modem_clock_domain_power_state_icg_map_init();
}

/// SOC Reset Reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
pub enum SocResetReason {
    /// Power on reset
    ///
    /// In ESP-IDF this value (0x01) can *also* be `ChipBrownOut` or
    /// `ChipSuperWdt`, however that is not really compatible with Rust-style
    /// enums.
    ChipPowerOn   = 0x01,
    /// Software resets the digital core
    CoreSw        = 0x03,
    /// Deep sleep reset the digital core (also: PMU HP power down core reset)
    CoreDeepSleep = 0x05,
    /// PMU HP CPU power down reset
    CpuPmuPwrDown = 0x06,
    /// Main watch dog 0 resets digital core
    CoreMwdt0     = 0x07,
    /// Main watch dog 1 resets digital core
    CoreMwdt1     = 0x08,
    /// RWDT core reset
    CoreRwdt      = 0x09,
    /// MWDT HP CPU reset
    CpuMwdt       = 0x0B,
    /// Software resets HP CPU
    CpuSw         = 0x0C,
    /// RWDT resets digital core
    CpuRwdt       = 0x0D,
    /// VDD voltage is not stable and resets the digital core
    SysBrownOut   = 0x0F,
    /// RWDT system reset
    SysRwdt       = 0x10,
    /// Super watch dog resets the digital core and rtc module
    SysSuperWdt   = 0x12,
    /// Glitch on power resets the digital core and rtc module
    CorePwrGlitch = 0x13,
    /// eFuse CRC error resets the digital core
    CoreEfuseCrc  = 0x14,
    /// USB Serial/JTAG controller's JTAG resets the digital core
    CoreUsbJtag   = 0x16,
    /// USB Serial/JTAG controller's UART resets the digital core
    CoreUsbUart   = 0x17,
    /// JTAG resets the CPU
    CpuJtag       = 0x18,
    /// CPU lockup (exception inside exception handler)
    CpuLockup     = 0x1A,
}

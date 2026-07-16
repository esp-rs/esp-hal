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

#[derive(Clone, Copy)]
struct HpSystemPower {
    dig_power: u32,
    clk_power: u32,
    xtal: u32,
}

#[derive(Clone, Copy)]
struct HpSystemClock {
    icg_func: [u32; 2],
    icg_apb: [u32; 2],
    icg_modem: u32,
    sysclk: u32,
}

#[derive(Clone, Copy)]
struct HpSystemDigital {
    syscntl: u32,
}

#[derive(Clone, Copy)]
struct HpSystemAnalog {
    bias: u32,
    regulator0: u32,
    regulator1: u32,
}

#[derive(Clone, Copy)]
struct HpSystemRetention {
    retention: u32,
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
    const fn active() -> Self {
        Self {
            power: HpSystemPower {
                dig_power: 0,
                clk_power: (1 << 22)
                    | (1 << 23)
                    | (1 << 24)
                    | (1 << 26)
                    | (1 << 27)
                    | (1 << 28)
                    | (1 << 30),
                xtal: 1 << 31,
            },
            clock: HpSystemClock {
                icg_func: [u32::MAX; 2],
                icg_apb: [u32::MAX; 2],
                icg_modem: 2 << 30,
                sysclk: 1 << 27,
            },
            digital: HpSystemDigital { syscntl: 1 << 23 },
            analog: HpSystemAnalog {
                bias: 1 << 25,
                regulator0: (1 << 3)
                    | (0xd << 4)
                    | (0x1c << 9)
                    | (1 << 14)
                    | (1 << 18)
                    | (HP_CALI_DBIAS << 27),
                regulator1: 0,
            },
            retention: HpSystemRetention {
                retention: (2 << 4)
                    | (2 << 6)
                    | (hp_retention_regdma_config(0, 0) << 18)
                    | (hp_retention_regdma_config(0, 2) << 23),
                backup_clk: [u32::MAX; 2],
            },
        }
    }

    // PMU_HP_MODEM_*_CONFIG_DEFAULT
    const fn modem() -> Self {
        Self {
            power: HpSystemPower {
                dig_power: 0,
                clk_power: (1 << 20)
                    | (1 << 21)
                    | (1 << 22)
                    | (1 << 23)
                    | (1 << 24)
                    | (1 << 27)
                    | (1 << 28),
                xtal: 1 << 31,
            },
            clock: HpSystemClock {
                // PMU_ICG_FUNC_ENA_ETM=40 and PMU_ICG_FUNC_ENA_BUS=47.
                icg_func: [0, (1 << (40 - 32)) | (1 << (47 - 32))],
                icg_apb: [0; 2],
                icg_modem: 1 << 30,
                // The S31 IDF deliberately keeps the modem-state system clock on XTAL.
                sysclk: (1 << 27) | (1 << 28) | (1 << 29),
            },
            digital: HpSystemDigital {
                syscntl: (1 << 23) | (1 << 25) | (1 << 26) | (1 << 28) | (1 << 29),
            },
            analog: HpSystemAnalog {
                bias: 0,
                regulator0: (1 << 18) | (HP_CALI_DBIAS << 27),
                regulator1: 0,
            },
            retention: HpSystemRetention {
                retention: (1 << 4) | (hp_retention_regdma_config(0, 1) << 20),
                backup_clk: [u32::MAX; 2],
            },
        }
    }

    // PMU_HP_SLEEP_*_CONFIG_DEFAULT
    const fn sleep() -> Self {
        Self {
            power: HpSystemPower {
                dig_power: 0,
                clk_power: (1 << 20) | (1 << 21),
                xtal: 0,
            },
            clock: HpSystemClock {
                icg_func: [0; 2],
                icg_apb: [0; 2],
                icg_modem: 0,
                sysclk: (1 << 28) | (1 << 29),
            },
            digital: HpSystemDigital {
                syscntl: (1 << 24) | (1 << 25) | (1 << 26) | (1 << 28) | (1 << 29),
            },
            analog: HpSystemAnalog {
                bias: (1 << 30) | (1 << 31),
                regulator0: 1 << 18,
                regulator1: 0,
            },
            retention: HpSystemRetention {
                retention: (2 << 8)
                    | (hp_retention_regdma_config(1, 1) << 20)
                    | (hp_retention_regdma_config(1, 0) << 25),
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
                pmu.[<$state _dig_power>]().write(|w| w.bits(param.power.dig_power));
                pmu.[<$state _hp_ck_power>]().write(|w| w.bits(param.power.clk_power));
                pmu.[<$state _xtal>]().write(|w| w.bits(param.power.xtal));

                pmu.[<$state _icg_hp_func>]().write(|w| w.bits(param.clock.icg_func[0]));
                pmu.[<$state _icg_hp_apb>]().write(|w| w.bits(param.clock.icg_apb[0]));
                pmu.[<$state _icg_hp_func1>]().write(|w| w.bits(param.clock.icg_func[1]));
                pmu.[<$state _icg_hp_apb1>]().write(|w| w.bits(param.clock.icg_apb[1]));
                pmu.[<$state _icg_modem>]().write(|w| w.bits(param.clock.icg_modem));
                pmu.[<$state _sysclk>]().write(|w| w.bits(param.clock.sysclk));

                pmu.[<$state _hp_sys_cntl>]().write(|w| w.bits(param.digital.syscntl));

                pmu.[<$state _bias>]().write(|w| w.bits(param.analog.bias));
                pmu.[<$state _hp_regulator0>]().write(|w| w.bits(param.analog.regulator0));
                pmu.[<$state _hp_regulator1>]().write(|w| w.bits(param.analog.regulator1));

                pmu.[<$state _backup>]().write(|w| w.bits(param.retention.retention));
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

#[derive(Clone, Copy)]
struct LpSystemPower {
    dig_power: u32,
    clk_power: u32,
    xtal: u32,
}

#[derive(Clone, Copy)]
struct LpSystemAnalog {
    bias: u32,
    regulator0: u32,
    regulator1: u32,
}

#[derive(Clone, Copy)]
struct LpSystemInit {
    power: LpSystemPower,
    analog: LpSystemAnalog,
}

impl LpSystemInit {
    // PMU_LP_ACTIVE_*_CONFIG_DEFAULT
    const fn active() -> Self {
        Self {
            power: LpSystemPower {
                dig_power: 0,
                clk_power: 1 << 30,
                xtal: 0,
            },
            analog: LpSystemAnalog {
                bias: 0,
                regulator0: (1 << 22) | (LP_CALI_DBIAS << 27),
                regulator1: 0,
            },
        }
    }

    // PMU_LP_SLEEP_*_CONFIG_DEFAULT
    const fn sleep() -> Self {
        Self {
            power: LpSystemPower {
                dig_power: 0,
                clk_power: 0,
                xtal: 0,
            },
            analog: LpSystemAnalog {
                bias: (1 << 30) | (1 << 31),
                regulator0: 1 << 22,
                regulator1: 0,
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
                .write(|w| w.bits(active.power.dig_power));
            pmu.hp_sleep_lp_ck_power()
                .write(|w| w.bits(active.power.clk_power));
            pmu.hp_sleep_lp_regulator0()
                .write(|w| w.bits(active.analog.regulator0));
            pmu.hp_sleep_lp_regulator1()
                .write(|w| w.bits(active.analog.regulator1));

            pmu.lp_sleep_lp_dig_power()
                .write(|w| w.bits(sleep.power.dig_power));
            pmu.lp_sleep_lp_ck_power()
                .write(|w| w.bits(sleep.power.clk_power));
            pmu.lp_sleep_bias().write(|w| w.bits(sleep.analog.bias));
            pmu.lp_sleep_lp_regulator0()
                .write(|w| w.bits(sleep.analog.regulator0));
            pmu.lp_sleep_lp_regulator1()
                .write(|w| w.bits(sleep.analog.regulator1));
            pmu.lp_sleep_xtal().write(|w| w.bits(sleep.power.xtal));
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

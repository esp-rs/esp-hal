//! Light- and deep-sleep support for the ESP32-S31.
use core::{
    ops::Not,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use crate::{
    peripherals::{
        EFUSE,
        HP_ALIVE_SYS,
        HP_SYS_CLKRST,
        IOMUX_MSPI_PIN,
        LP_AON_CLK_RST,
        LP_SYS as LP_AON,
        MEMSPI2,
        PMU,
    },
    private::DropGuard,
    rtc_cntl::{
        Rtc,
        rtc::{HpAnalog, HpSysCntlReg, HpSysPower, LpAnalog, LpSysPower},
        sleep::{
            SleepKind,
            pmu_common::{SleepTimeConfig, request_sleep},
        },
    },
    soc::{
        clocks::{self, ClockTree, CpuRootClkConfig},
        xtal32k,
    },
};

const HP_CALI_DBIAS: u8 = 26;
const LP_CALI_DBIAS: u8 = 26;

/// Configuration for controlling analog behavior during sleep.
#[derive(Clone, Copy)]
pub struct AnalogSleepConfig {
    /// High-power system analog configuration.
    pub hp_sys: HpAnalog,
    /// Low-power system analog configuration.
    pub lp_sys_sleep: LpAnalog,
}

impl AnalogSleepConfig {
    fn defaults_deep_sleep() -> Self {
        // PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT
        Self {
            hp_sys: {
                let mut cfg = HpAnalog::default();
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_xpd(false);
                cfg.bias.set_dbg_atten(0);
                cfg
            },
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();
                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(12);
                cfg.regulator0.set_dbias(23);
                cfg
            },
        }
    }

    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        let mut this = Self {
            hp_sys: {
                let mut cfg = HpAnalog::default();
                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(0);
                cfg.regulator0.set_dbias(1);
                cfg
            },
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();
                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(0);
                cfg.regulator0.set_dbias(12);
                cfg
            },
        };

        // Like esp-idf's `pmu_sleep_config_default`, this code keeps the reference current on
        // while PSRAM is mapped, see
        // <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/port/esp32s31/pmu_sleep.c#L223-L229>.
        // Without the current, PSRAM loses its data in light sleep. It also loses the latencies
        // that `Psram::new` wrote to its mode registers.
        if !pd_flags.pd_xtal() || !crate::psram::psram_range().is_empty() {
            this.hp_sys.bias.set_pd_cur(false);
            this.lp_sys_sleep.bias.set_pd_cur(false);
        }

        if !pd_flags.pd_xtal() || !pd_flags.pd_rc_fast() {
            this.hp_sys.bias.set_dbg_atten(0);
            this.hp_sys.regulator0.set_dbias(HP_CALI_DBIAS);
            this.lp_sys_sleep.bias.set_dbg_atten(0);
            this.lp_sys_sleep.regulator0.set_dbias(LP_CALI_DBIAS);
        }

        this
    }

    fn apply(&self) {
        // pmu_sleep_analog_init
        PMU::regs().hp_sleep_bias().modify(|_, w| unsafe {
            w.hp_sleep_dbg_atten().bits(self.hp_sys.bias.dbg_atten());
            w.hp_sleep_pd_cur().bit(self.hp_sys.bias.pd_cur());
            w.sleep().bit(self.hp_sys.bias.bias_sleep())
        });
        PMU::regs().hp_sleep_hp_regulator0().modify(|_, w| unsafe {
            w.hp_sleep_hp_regulator_slp_mem_xpd()
                .bit(self.hp_sys.regulator0.slp_mem_xpd());
            w.hp_sleep_hp_regulator_slp_logic_xpd()
                .bit(self.hp_sys.regulator0.slp_logic_xpd());
            w.hp_sleep_hp_regulator_xpd()
                .bit(self.hp_sys.regulator0.xpd());
            w.hp_sleep_hp_regulator_slp_logic_dbias()
                .bits(self.hp_sys.regulator0.slp_logic_dbias());
            w.hp_sleep_hp_regulator_dbias()
                .bits(self.hp_sys.regulator0.dbias())
        });
        PMU::regs().hp_sleep_hp_regulator1().modify(|_, w| unsafe {
            w.hp_sleep_hp_regulator_drv_b()
                .bits(self.hp_sys.regulator1.drv_b())
        });

        PMU::regs().lp_sleep_bias().modify(|_, w| unsafe {
            w.lp_sleep_dbg_atten()
                .bits(self.lp_sys_sleep.bias.dbg_atten());
            w.lp_sleep_pd_cur().bit(self.lp_sys_sleep.bias.pd_cur());
            w.sleep().bit(self.lp_sys_sleep.bias.bias_sleep())
        });
        PMU::regs().lp_sleep_lp_regulator0().modify(|_, w| unsafe {
            w.lp_sleep_lp_regulator_slp_xpd()
                .bit(self.lp_sys_sleep.regulator0.slp_xpd());
            w.lp_sleep_lp_regulator_xpd()
                .bit(self.lp_sys_sleep.regulator0.xpd());
            w.lp_sleep_lp_regulator_slp_dbias()
                .bits(self.lp_sys_sleep.regulator0.slp_dbias());
            w.lp_sleep_lp_regulator_dbias()
                .bits(self.lp_sys_sleep.regulator0.dbias())
        });
        PMU::regs().lp_sleep_lp_regulator1().modify(|_, w| unsafe {
            w.lp_sleep_lp_regulator_drv_b()
                .bits(self.lp_sys_sleep.regulator1.drv_b())
        });
    }
}

/// Configuration for digital peripherals during sleep.
#[derive(Clone, Copy)]
pub struct DigitalSleepConfig {
    /// High-power system control register configuration.
    pub syscntl: HpSysCntlReg,
}

impl DigitalSleepConfig {
    fn defaults_deep_sleep(pd_flags: PowerDownFlags) -> Self {
        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_dig_pad_slp_sel(false);
        syscntl.set_lp_pad_hold_all(true);
        syscntl.set_hp_pad_hold_all(true);
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_c_channel(!pd_flags.pd_top());
        Self { syscntl }
    }

    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_dig_pad_slp_sel(!pd_flags.pd_top());
        syscntl.set_lp_pad_hold_all(pd_flags.pd_top());
        syscntl.set_hp_pad_hold_all(pd_flags.pd_top());
        syscntl.set_dig_pause_wdt(true);
        syscntl.set_c_channel(!pd_flags.pd_top());
        Self { syscntl }
    }

    fn apply(&self) {
        PMU::regs().hp_sleep_hp_sys_cntl().modify(|_, w| {
            w.hp_sleep_c_channel().bit(self.syscntl.c_channel());
            w.hp_sleep_dig_pad_slp_sel()
                .bit(self.syscntl.dig_pad_slp_sel());
            w.hp_sleep_lp_pad_hold_all()
                .bit(self.syscntl.lp_pad_hold_all());
            w.hp_sleep_hp_pad_hold_all()
                .bit(self.syscntl.hp_pad_hold_all());
            w.hp_sleep_dig_pause_wdt().bit(self.syscntl.dig_pause_wdt());
            w.hp_sleep_dig_cpu_stall().bit(true)
        });
    }
}

/// Configuration of HP and LP power domains during sleep.
#[derive(Clone, Copy)]
pub struct PowerSleepConfig {
    /// Power configuration for the high-power system during sleep.
    pub hp_sys: HpSysPower,
    /// Power configuration for the low-power system when it is active.
    pub lp_sys_active: LpSysPower,
    /// Power configuration for the low-power system when it is in sleep mode.
    pub lp_sys_sleep: LpSysPower,
}

impl PowerSleepConfig {
    fn defaults(pd_flags: PowerDownFlags) -> Self {
        let mut this = Self {
            hp_sys: HpSysPower::default(),
            lp_sys_active: LpSysPower::default(),
            lp_sys_sleep: LpSysPower::default(),
        };
        this.apply_flags(pd_flags);
        this
    }

    fn apply_flags(&mut self, pd_flags: PowerDownFlags) {
        // PMU_SLEEP_POWER_CONFIG_DEFAULT
        self.hp_sys
            .dig_power
            .set_vdd_spi_pd_en(pd_flags.pd_vddsdio());
        self.hp_sys
            .dig_power
            .set_hp_alive_pd_en(pd_flags.pd_mem() || pd_flags.pd_hp_aon());
        self.hp_sys
            .dig_power
            .set_modem_top_pd_en(pd_flags.pd_modem());
        self.hp_sys.dig_power.set_hp_cnnt_pd_en(pd_flags.pd_modem());
        self.hp_sys.dig_power.set_hp_cpu_pd_en(pd_flags.pd_cpu());
        self.hp_sys.dig_power.set_modem_pwr_pd_en(pd_flags.pd_mem());
        self.hp_sys.dig_power.set_top_pd_en(pd_flags.pd_top());

        self.hp_sys.clk.set_i2c_iso_en(true);
        self.hp_sys.clk.set_i2c_retention(true);

        self.hp_sys.xtal.set_xpd_xtal(pd_flags.pd_xtal().not());

        self.lp_sys_active
            .clk_power
            .set_xpd_xtal32k(pd_flags.pd_xtal32k().not());
        self.lp_sys_active
            .clk_power
            .set_xpd_rc32k(pd_flags.pd_rc32k().not());
        self.lp_sys_active.clk_power.set_xpd_fosc(true);

        self.lp_sys_sleep
            .dig_power
            .set_peri_pd_en(pd_flags.pd_lp_periph());

        self.lp_sys_sleep
            .clk_power
            .set_xpd_xtal32k(pd_flags.pd_xtal32k().not());
        self.lp_sys_sleep
            .clk_power
            .set_xpd_rc32k(pd_flags.pd_rc32k().not());
        self.lp_sys_sleep
            .clk_power
            .set_xpd_fosc(pd_flags.pd_rc_fast().not());

        self.lp_sys_sleep
            .xtal
            .set_xpd_xtal(pd_flags.pd_xtal().not());
    }

    fn apply(&self, dslp: bool) {
        // pmu_sleep_power_init
        PMU::regs()
            .hp_sleep_dig_power()
            .modify(|_, w| unsafe { w.bits(self.hp_sys.dig_power.0) });
        PMU::regs()
            .hp_sleep_hp_ck_power()
            .modify(|_, w| unsafe { w.bits(self.hp_sys.clk.0) });
        PMU::regs()
            .hp_sleep_xtal()
            .modify(|_, w| w.hp_sleep_xpd_xtal().bit(self.hp_sys.xtal.xpd_xtal()));

        set_memory_power_on_mask(!dslp);

        PMU::regs()
            .hp_sleep_lp_dig_power()
            .modify(|_, w| unsafe { w.bits(self.lp_sys_active.dig_power.0) });
        PMU::regs()
            .hp_sleep_lp_ck_power()
            .modify(|_, w| unsafe { w.bits(self.lp_sys_active.clk_power.0) });

        PMU::regs()
            .lp_sleep_lp_dig_power()
            .modify(|_, w| unsafe { w.bits(self.lp_sys_sleep.dig_power.0) });
        PMU::regs()
            .lp_sleep_lp_ck_power()
            .modify(|_, w| unsafe { w.bits(self.lp_sys_sleep.clk_power.0) });
        PMU::regs()
            .lp_sleep_xtal()
            .modify(|_, w| w.lp_sleep_xpd_xtal().bit(self.lp_sys_sleep.xtal.xpd_xtal()));
    }
}

fn set_memory_power_on_mask(keep_on: bool) {
    // pmu_ll_hp_set_memory_power_on_mask: 0 in deep sleep, 0xf in light sleep.
    let mask = if keep_on { 0x1f } else { 0 };
    PMU::regs().power_pd_mem_mask().modify(|_, w| unsafe {
        w.pd_hp_mem0_mask().bits(mask);
        w.pd_hp_mem1_mask().bits(mask);
        w.pd_hp_mem2_mask().bits(mask)
    });
    PMU::regs().power_pd_mem_cntl().modify(|r, w| unsafe {
        let mut pu = r.force_hp_mem_pu().bits();
        if keep_on {
            pu |= 1 << 3;
        } else {
            pu &= !(1 << 3);
        }
        w.force_hp_mem_pu().bits(pu)
    });
}

/// High-power system sleep timing parameters.
#[derive(Clone, Copy, Default)]
pub struct HpParam {
    analog_wait_target_cycle: u16,
    digital_power_supply_wait_cycle: u16,
    digital_power_up_wait_cycle: u16,
    pll_stable_wait_cycle: u16,
    modem_wakeup_wait_cycle: u32,
    isolate_wait_cycle: u8,
    reset_wait_cycle: u8,
    min_slp_slow_clk_cycle: u8,
}

/// Low-power system sleep timing parameters.
#[derive(Clone, Copy, Default)]
pub struct LpParam {
    digital_power_supply_wait_cycle: u16,
    min_slp_slow_clk_cycle: u8,
    analog_wait_target_cycle: u8,
    digital_power_up_wait_cycle: u8,
    isolate_wait_cycle: u8,
    reset_wait_cycle: u8,
}

/// Shared HP/LP sleep timing parameters.
#[derive(Clone, Copy, Default)]
pub struct HpLpParam {
    xtal_stable_wait_cycle: u16,
}

/// Sleep timing parameter configuration.
#[derive(Clone, Copy)]
pub struct ParamSleepConfig {
    hp_sys: HpParam,
    lp_sys: LpParam,
    hp_lp: HpLpParam,
}

impl ParamSleepConfig {
    fn apply(&self) {
        // pmu_sleep_param_init
        PMU::regs().slp_wakeup_cntl3().modify(|_, w| unsafe {
            w.hp_min_slp_val().bits(self.hp_sys.min_slp_slow_clk_cycle);
            w.lp_min_slp_val().bits(self.lp_sys.min_slp_slow_clk_cycle)
        });

        PMU::regs().slp_wakeup_cntl7().modify(|_, w| unsafe {
            w.ana_wait_target()
                .bits(self.hp_sys.analog_wait_target_cycle)
        });

        PMU::regs().power_wait_timer0().modify(|_, w| unsafe {
            w.dg_hp_pd_wait_timer()
                .bits(self.hp_sys.digital_power_supply_wait_cycle);
            w.dg_hp_powerup_timer()
                .bits(self.hp_sys.digital_power_up_wait_cycle)
        });

        PMU::regs().power_wait_timer1().modify(|_, w| unsafe {
            w.dg_lp_pd_wait_timer()
                .bits(self.lp_sys.digital_power_supply_wait_cycle);
            w.dg_lp_powerup_timer()
                .bits(self.lp_sys.digital_power_up_wait_cycle)
        });

        PMU::regs().power_wait_timer2().modify(|_, w| unsafe {
            w.dg_hp_iso_wait_timer()
                .bits(self.hp_sys.isolate_wait_cycle);
            w.dg_hp_rst_wait_timer().bits(self.hp_sys.reset_wait_cycle);
            w.dg_lp_iso_wait_timer()
                .bits(self.lp_sys.isolate_wait_cycle);
            w.dg_lp_rst_wait_timer().bits(self.lp_sys.reset_wait_cycle)
        });

        PMU::regs().slp_wakeup_cntl5().modify(|_, w| unsafe {
            w.lp_ana_wait_target()
                .bits(self.lp_sys.analog_wait_target_cycle);
            w.modem_wait_target()
                .bits(self.hp_sys.modem_wakeup_wait_cycle)
        });

        PMU::regs().power_ck_wait_cntl().modify(|_, w| unsafe {
            w.pmu_wait_xtl_stable()
                .bits(self.hp_lp.xtal_stable_wait_cycle);
            w.pmu_wait_pll_stable()
                .bits(self.hp_sys.pll_stable_wait_cycle)
        });
    }

    fn defaults(config: SleepTimeConfig, pd_flags: PowerDownFlags, xpd_xtal: bool) -> Self {
        let hw_wait_time_us = config.pmu_sleep_calculate_hw_wait_time(pd_flags);
        let modem_wakeup_wait_time_us = (config.sleep_time_adjustment
            + MachineConstants::MODEM_STATE_SKIP_TIME_US
            + MachineConstants::HP_REGDMA_RF_ON_WORK_TIME_US)
            .saturating_sub(hw_wait_time_us);

        let hp_sys = HpParam {
            min_slp_slow_clk_cycle: config.us_to_slowclk(MachineConstants::HP_MIN_SLP_TIME_US)
                as u8,
            analog_wait_target_cycle: config.us_to_fastclk(MachineConstants::HP_ANALOG_WAIT_TIME_US)
                as u16,
            digital_power_supply_wait_cycle: config
                .us_to_fastclk(MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US)
                as u16,
            digital_power_up_wait_cycle: config
                .us_to_fastclk(MachineConstants::HP_POWER_UP_WAIT_TIME_US)
                as u16,
            pll_stable_wait_cycle: config
                .us_to_fastclk(MachineConstants::HP_PLL_WAIT_STABLE_TIME_US)
                as u16,
            modem_wakeup_wait_cycle: config.us_to_fastclk(modem_wakeup_wait_time_us),
            isolate_wait_cycle: config.us_to_fastclk(MachineConstants::HP_ISOLATE_WAIT_TIME_US)
                as u8,
            reset_wait_cycle: config.us_to_fastclk(MachineConstants::HP_RESET_WAIT_TIME_US) as u8,
        };

        let lp_sys = LpParam {
            min_slp_slow_clk_cycle: config.us_to_slowclk(MachineConstants::LP_MIN_SLP_TIME_US)
                as u8,
            analog_wait_target_cycle: config.us_to_slowclk(MachineConstants::LP_ANALOG_WAIT_TIME_US)
                as u8,
            digital_power_supply_wait_cycle: config
                .us_to_fastclk(MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US)
                as u16,
            digital_power_up_wait_cycle: config
                .us_to_fastclk(MachineConstants::LP_POWER_UP_WAIT_TIME_US)
                as u8,
            isolate_wait_cycle: config.us_to_fastclk(MachineConstants::LP_ISOLATE_WAIT_TIME_US)
                as u8,
            reset_wait_cycle: config.us_to_fastclk(MachineConstants::LP_RESET_WAIT_TIME_US) as u8,
        };

        let xtal_stable_wait_cycle = if xpd_xtal {
            config.us_to_slowclk(MachineConstants::LP_XTAL_WAIT_STABLE_TIME_US) as u16
        } else {
            config.us_to_fastclk(MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US) as u16
        };

        Self {
            hp_sys,
            lp_sys,
            hp_lp: HpLpParam {
                xtal_stable_wait_cycle,
            },
        }
    }
}

impl SleepTimeConfig {
    pub(crate) const CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: u32 = 160;
    pub(crate) const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 780;

    pub(crate) fn pmu_sleep_calculate_hw_wait_time(&self, pd_flags: PowerDownFlags) -> u32 {
        let lp_wakeup_wait_time_us = self.slowclk_to_us(MachineConstants::LP_WAKEUP_WAIT_CYCLE);
        let lp_clk_switch_time_us = self.slowclk_to_us(MachineConstants::LP_CLK_SWITCH_CYCLE);
        let lp_clk_power_on_wait_time_us =
            self.slowclk_to_us(MachineConstants::LP_CLK_POWER_ON_WAIT_CYCLE);
        let lp_control_wait_time_us =
            MachineConstants::LP_ISOLATE_WAIT_TIME_US + MachineConstants::LP_RESET_WAIT_TIME_US;

        let lp_hw_wait_time_us = MachineConstants::LP_MIN_SLP_TIME_US
            + MachineConstants::LP_ANALOG_WAIT_TIME_US
            + lp_clk_power_on_wait_time_us
            + lp_wakeup_wait_time_us
            + lp_clk_switch_time_us
            + MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::LP_POWER_UP_WAIT_TIME_US
            + lp_control_wait_time_us;

        let hp_digital_power_up_wait_time_us = MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::HP_POWER_UP_WAIT_TIME_US;
        let hp_control_wait_time_us =
            MachineConstants::HP_ISOLATE_WAIT_TIME_US + MachineConstants::HP_RESET_WAIT_TIME_US;
        // The sleep backs up no register with REGDMA, so the wake needs no restore time.
        let hp_regdma_wait_time_us = 0;
        let hp_clock_wait_time_us = if pd_flags.pd_xtal() {
            MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US
                + MachineConstants::HP_PLL_WAIT_STABLE_TIME_US
        } else {
            MachineConstants::HP_PLL_WAIT_STABLE_TIME_US
        };

        let hp_hw_wait_time_us = MachineConstants::HP_ANALOG_WAIT_TIME_US
            + hp_digital_power_up_wait_time_us
            + hp_regdma_wait_time_us
            + hp_clock_wait_time_us
            + hp_control_wait_time_us;

        lp_hw_wait_time_us + hp_hw_wait_time_us
    }
}

/// Configuration for the RTC sleep behavior.
#[derive(Clone, Copy)]
pub struct RtcSleepConfig {
    /// Deep sleep flag.
    pub deep: bool,
    /// Power-down flags.
    pub pd_flags: PowerDownFlags,
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        Self {
            deep: false,
            pd_flags: PowerDownFlags(0),
        }
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    /// Power domains to be powered down during sleep.
    pub struct PowerDownFlags(u32);

    /// Controls the power-down status of the top power domain.
    pub u32, pd_top      , set_pd_top      : 0;
    /// Controls the power-down status of the VDD_SDIO power domain.
    pub u32, pd_vddsdio  , set_pd_vddsdio  : 1;
    /// Controls the power-down status of the modem / connectivity power domain.
    pub u32, pd_modem    , set_pd_modem    : 2;
    /// Controls the power-down status of the high-performance peripheral power domain.
    pub u32, pd_hp_periph, set_pd_hp_periph: 3;
    /// Controls the power-down status of the CPU power domain.
    ///
    /// Crate-private, because a light sleep needs CPU retention to power this domain down. A
    /// power-down without retention loses the CPU state.
    pub(crate) u32, pd_cpu, set_pd_cpu: 4;
    /// Controls the power-down status of the high-performance always-on domain.
    pub u32, pd_hp_aon   , set_pd_hp_aon   : 5;
    /// Controls the power-down status of memory group 0.
    pub u32, pd_mem_g0   , set_pd_mem_g0   : 6;
    /// Controls the power-down status of memory group 1.
    pub u32, pd_mem_g1   , set_pd_mem_g1   : 7;
    /// Controls the power-down status of memory group 2.
    pub u32, pd_mem_g2   , set_pd_mem_g2   : 8;
    /// Controls the power-down status of memory group 3.
    pub u32, pd_mem_g3   , set_pd_mem_g3   : 9;
    /// Controls the power-down status of the crystal oscillator.
    pub u32, pd_xtal     , set_pd_xtal     : 10;
    /// Controls the power-down status of the fast RC oscillator.
    pub u32, pd_rc_fast  , set_pd_rc_fast  : 11;
    /// Controls the power-down status of the 32 kHz crystal oscillator.
    pub u32, pd_xtal32k  , set_pd_xtal32k  : 12;
    /// Controls the power-down status of the 32 kHz RC oscillator.
    pub u32, pd_rc32k    , set_pd_rc32k    : 13;
    /// Controls the power-down status of the low-power peripheral domain.
    pub u32, pd_lp_periph, set_pd_lp_periph: 14;
}

impl PowerDownFlags {
    /// Returns whether all memory groups are powered down.
    pub fn pd_mem(self) -> bool {
        self.pd_mem_g0() && self.pd_mem_g1() && self.pd_mem_g2() && self.pd_mem_g3()
    }

    /// Sets the power-down status for all memory groups at once.
    pub fn set_pd_mem(&mut self, value: bool) {
        self.set_pd_mem_g0(value);
        self.set_pd_mem_g1(value);
        self.set_pd_mem_g2(value);
        self.set_pd_mem_g3(value);
    }
}

// Constants from `PMU_SLEEP_MC_DEFAULT()` in esp-idf pmu_param.h.
struct MachineConstants;
impl MachineConstants {
    const LP_MIN_SLP_TIME_US: u32 = 450;
    const LP_WAKEUP_WAIT_CYCLE: u32 = 4;
    const LP_ANALOG_WAIT_TIME_US: u32 = 154;
    const LP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const LP_CLK_SWITCH_CYCLE: u32 = 1;
    const LP_CLK_POWER_ON_WAIT_CYCLE: u32 = 1;
    const LP_ISOLATE_WAIT_TIME_US: u32 = 1;
    const LP_RESET_WAIT_TIME_US: u32 = 1;
    const LP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    const LP_POWER_UP_WAIT_TIME_US: u32 = 2;

    const HP_MIN_SLP_TIME_US: u32 = 450;
    const HP_ANALOG_WAIT_TIME_US: u32 = 154;
    const HP_ISOLATE_WAIT_TIME_US: u32 = 1;
    const HP_RESET_WAIT_TIME_US: u32 = 1;
    const HP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    const HP_POWER_UP_WAIT_TIME_US: u32 = 2;
    const HP_REGDMA_M2A_WORK_TIME_US: u32 = 278;
    const HP_REGDMA_RF_ON_WORK_TIME_US: u32 = 70;
    const HP_SYSTEM_DFS_UP_WORK_TIME_US: u32 = 124;
    const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const HP_PLL_WAIT_STABLE_TIME_US: u32 = 50;

    const MODEM_STATE_SKIP_TIME_US: u32 = Self::HP_REGDMA_M2A_WORK_TIME_US
        + Self::HP_SYSTEM_DFS_UP_WORK_TIME_US
        + Self::LP_MIN_SLP_TIME_US;
}

impl RtcSleepConfig {
    /// Returns whether the device is in deep sleep mode.
    pub fn deep_slp(&self) -> bool {
        self.deep
    }

    /// Configures the device for deep sleep mode.
    pub fn deep() -> Self {
        Self {
            deep: true,
            ..Self::default()
        }
    }

    pub(crate) fn is_deep_sleep(&self) -> bool {
        self.deep
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {}

    /// Selects the kind of the sleep, and what the sleep powers down.
    ///
    /// Sleep entry calls this before the hooks of the wakeup sources, so that a source can keep
    /// powered what it needs.
    pub(crate) fn set_sleep_kind(&mut self, kind: SleepKind) {
        self.deep = kind == SleepKind::Deep;

        let lp_slow_uses_xtal32k = cfg_select! {
            use_xtal32k => ClockTree::with(|clocks| {
                matches!(
                    clocks::lp_slow_clk_config(clocks),
                    Some(clocks::LpSlowClkConfig::Xtal32k)
                )
            }),
            _ => false,
        };

        if self.deep {
            self.pd_flags.set_pd_top(true);
            self.pd_flags.set_pd_vddsdio(true);
            self.pd_flags.set_pd_modem(true);
            self.pd_flags.set_pd_hp_periph(true);
            self.pd_flags.set_pd_cpu(true);
            self.pd_flags.set_pd_mem(true);
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_hp_aon(true);
            self.pd_flags.set_pd_lp_periph(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
            self.pd_flags.set_pd_rc32k(true);
            self.pd_flags.set_pd_rc_fast(true);
        } else {
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags
                .set_pd_xtal32k(!lp_slow_uses_xtal32k && !xtal32k::use_xtal32k());
        }
    }

    /// Configures the wakeup and reject sources of the sleep.
    ///
    /// [`Self::enter_sleep`] requests the sleep after this call. The return value is a guard that
    /// restores what sleep entry changed for the sleep only, so the caller keeps it until the
    /// sleep ends.
    #[crate::ram]
    pub(crate) fn start_sleep(&self, wakeup_mask: u32, reject_mask: u32) -> impl Sized {
        let restore_clock_config = ClockTree::with(|clocks| {
            let old_cpu_root_clk = clocks.cpu_root_clk();

            // Like esp-idf's `esp_sleep_start`, this code slows PSRAM before the CPU moves to
            // XTAL_CLK, see
            // <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/sleep_modes.c#L1036-L1043>.
            // The guard restores the PSRAM speed after the CPU moves back, as esp-idf does at
            // <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/sleep_modes.c#L1195>.
            // esp-idf also clears the PSRAM tuning registers for the low speed. esp-hal does not
            // tune PSRAM or flash, so these registers keep their reset value at both speeds.
            let psram_slowed = !self.deep
                && crate::psram::slow_in_light_sleep()
                && crate::psram::low_speed::enter();

            clocks::configure_cpu_root_clk(clocks, CpuRootClkConfig::Xtal);
            // `pmu_sleep_finish` runs after the wake with the caches suspended, and reads this
            // copy. The clock tree does not change while the chip sleeps.
            BBPLL_IN_CLOCK_TREE.store(clocks::bbpll_requested(clocks), Ordering::Relaxed);

            DropGuard::new((), move |_| {
                ClockTree::with(|clocks| {
                    if let Some(old) = old_cpu_root_clk {
                        clocks::configure_cpu_root_clk(clocks, old);
                    }
                });
                if psram_slowed {
                    crate::psram::low_speed::exit();
                }
            })
        });

        let power = PowerSleepConfig::defaults(self.pd_flags);
        power.apply(self.deep);

        let config = if self.deep {
            SleepTimeConfig::deep_sleep()
        } else {
            SleepTimeConfig::light_sleep(self.pd_flags)
        };

        let mut param =
            ParamSleepConfig::defaults(config, self.pd_flags, power.hp_sys.xtal.xpd_xtal());

        if self.deep {
            const PMU_LP_ANALOG_WAIT_TARGET_TIME_DSLP_US: u32 = 500;
            param.lp_sys.analog_wait_target_cycle =
                config.us_to_slowclk(PMU_LP_ANALOG_WAIT_TARGET_TIME_DSLP_US) as u8;

            DigitalSleepConfig::defaults_deep_sleep(self.pd_flags).apply();
            AnalogSleepConfig::defaults_deep_sleep().apply();
        } else {
            AnalogSleepConfig::defaults_light_sleep(self.pd_flags).apply();
            DigitalSleepConfig::defaults_light_sleep(self.pd_flags).apply();
        }

        param.apply();

        // lp_aon_hal_inform_wakeup_type: RTC_SLEEP_MODE_REG is LP_STORE8.
        LP_AON::regs()
            .lp_store(8)
            .modify(|r, w| unsafe { w.bits(r.bits() & !0x01 | self.deep as u32) });

        PMU::regs()
            .slp_wakeup_cntl2()
            .write(|w| unsafe { w.bits(wakeup_mask) });

        PMU::regs()
            .slp_wakeup_cntl1()
            .write(|w| unsafe { w.sleep_reject_ena().bits(reject_mask) });

        PMU::regs()
            .slp_wakeup_cntl4()
            .write(|w| w.slp_reject_cause_clr().bit(true));

        PMU::regs().int_clr().write(|w| {
            w.sw().clear_bit_by_one();
            w.soc_sleep_reject().clear_bit_by_one();
            w.soc_wakeup().clear_bit_by_one()
        });

        PMU::regs()
            .slp_wakeup_cntl0()
            .write(|w| w.slp_reject_en().bit(reject_mask != 0));

        restore_clock_config
    }

    /// Requests the sleep.
    ///
    /// The caller waits for the result of the request.
    #[inline(always)]
    pub(crate) fn enter_sleep(&self) -> bool {
        if self.deep {
            request_sleep()
        } else {
            pmu_sleep_start()
        }
    }

    /// Cleans up after sleep.
    #[inline(always)]
    pub(crate) fn finish_sleep(&self) {}
}

/// The MPLL frequency in MHz before [`pmu_sleep_start`] turned the MPLL off. It is 0 when the MPLL
/// was off. This value is a static, because a wake from a CPU power-down reaches
/// [`pmu_sleep_finish`] through the wake stub, not through a return from [`pmu_sleep_start`].
static MPLL_MHZ_BEFORE_SLEEP: AtomicU32 = AtomicU32::new(0);

/// Whether PSRAM was mapped when [`pmu_sleep_start`] ran.
static PSRAM_IN_SLEEP: AtomicBool = AtomicBool::new(false);

/// Whether the clock tree had the BBPLL on during the sleep.
static BBPLL_IN_CLOCK_TREE: AtomicBool = AtomicBool::new(false);

/// Holds the PSRAM pins at their levels, or releases them.
///
/// Ports esp-idf's `mspi_ll_psram_hold_all_pins` and `mspi_ll_psram_unhold_all_pins`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_mspi/esp32s31/include/hal/mspi_ll.h#L686-L720>.
#[inline(always)]
fn hold_psram_pins(hold: bool) {
    let pins = IOMUX_MSPI_PIN::regs();
    pins.psram_d_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_q_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_wp_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_hold_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_dq4_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_dq5_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_dq6_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_dq7_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_dqs_0_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_ck_pin0().modify(|_, w| w.hold().bit(hold));
    pins.psram_cs_pin0().modify(|_, w| w.hold().bit(hold));
}

/// Moves both PSRAM controllers to XTAL_CLK or back to the MPLL, and gates or ungates their
/// clocks. MSPI2 and MSPI3 share these fields.
///
/// Ports esp-idf's `_psram_ctrlr_ll_select_clk_source`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_mspi/esp32s31/include/hal/psram_ctrlr_ll.h#L414-L431>,
/// `_psram_ctrlr_ll_enable_core_clock`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_mspi/esp32s31/include/hal/psram_ctrlr_ll.h#L468-L471>,
/// and `_psram_ctrlr_ll_enable_module_clock`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_mspi/esp32s31/include/hal/psram_ctrlr_ll.h#L372-L377>.
#[inline(always)]
fn set_psram_clock_for_sleep(asleep: bool) {
    HP_SYS_CLKRST::regs().psram_ctrl0().modify(|_, w| unsafe {
        w.clk_src_sel().bits(if asleep { 0 } else { 1 });
        w.core_clk_en().bit(!asleep);
        w.sys_clk_en().bit(!asleep);
        w.pll_clk_en().bit(!asleep)
    });
}

/// Sets the modem's PLL clock gate.
///
/// Ports esp-idf's `_clk_gate_ll_modem_pll_source_cg_en`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_clock/esp32s31/include/hal/clk_gate_ll.h#L367-L376>.
#[inline(always)]
fn set_modem_pll_clock_gate(enable: bool) {
    let modem_conf = HP_SYS_CLKRST::regs().modem_conf();
    let (first, second) = if enable { (0x15, 0x1d) } else { (0x2d, 0x25) };
    modem_conf.write(|w| unsafe { w.bits(first) });
    modem_conf.write(|w| unsafe { w.bits(second) });
}

/// Powers the BBPLL down.
///
/// Ports esp-idf's `clk_ll_bbpll_disable`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_clock/esp32s31/include/hal/clk_tree_ll.h#L169-L175>.
#[inline(always)]
fn disable_bbpll() {
    PMU::regs()
        .imm_hp_ck_power_1()
        .modify(|_, w| w.tie_low_global_bbpll_icg().set_bit());
    PMU::regs().imm_hp_ck_power_1().modify(|_, w| {
        w.tie_low_xpd_bbpll().set_bit();
        w.tie_low_xpd_bbpll_i2c().set_bit()
    });
    HP_ALIVE_SYS::regs()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_spll_480m_clk_en().clear_bit());
    PMU::regs().hp_active_hp_ck_power().modify(|_, w| {
        w.hp_active_xpd_bbpll_i2c().clear_bit();
        w.hp_active_xpd_bbpll().clear_bit()
    });
}

/// Ports the PSRAM and MPLL steps of esp-idf's `pmu_sleep_start`, and requests a light sleep, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/port/esp32s31/pmu_sleep.c#L355-L398>.
///
/// Returns `true` if the hardware rejected the request. This function runs from RAM, because the
/// caches are suspended. A wake from a CPU power-down does not return here. The retention code
/// calls [`pmu_sleep_finish`] instead.
#[crate::ram]
pub(crate) fn pmu_sleep_start() -> bool {
    let psram = !crate::psram::psram_range().is_empty();
    PSRAM_IN_SLEEP.store(psram, Ordering::Relaxed);
    if psram {
        // `psram_ctrlr_ll_wait_all_transaction_done`: the six FIFO-empty bits, 26 to 31.
        const ALL_TRANSACTION_DONE: u32 = 0xfc00_0000;
        while MEMSPI2::regs().smem_axi_addr_ctrl().read().bits() != ALL_TRANSACTION_DONE {}
        hold_psram_pins(true);
    }

    let mpll_on = PMU::regs()
        .hp_active_hp_ck_power()
        .read()
        .hp_active_xpd_mpll()
        .bit_is_set();
    // esp-idf's `rtc_clk_mpll_get_freq` returns a software copy of the rate. This code reads the
    // dividers instead, as `clk_ll_mpll_get_freq_mhz` does, see
    // <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hal_clock/esp32s31/include/hal/clk_tree_ll.h#L488-L493>.
    let mpll_mhz = if mpll_on {
        let div = LP_AON_CLK_RST::regs().mspi_div().read();
        40 * (div.mspi_fb_div().bits() as u32 + 1) / (div.mspi_ref_div().bits() as u32 + 1)
    } else {
        0
    };
    MPLL_MHZ_BEFORE_SLEEP.store(mpll_mhz, Ordering::Relaxed);
    if mpll_on {
        // esp-idf gates the controller clocks when a REGDMA backup does not restore them. esp-hal
        // does not use REGDMA backups.
        if psram {
            set_psram_clock_for_sleep(true);
        }
        clocks::set_mpll_power(false);
    }

    let rejected = request_sleep();
    pmu_sleep_finish();
    rejected
}

/// Ports esp-idf's `pmu_sleep_finish`, see
/// <https://github.com/espressif/esp-idf/blob/383dda6c182/components/esp_hw_support/port/esp32s31/pmu_sleep.c#L409-L465>.
/// It undoes [`pmu_sleep_start`] after the sleep.
///
/// It runs before the caches resume, so PSRAM is back on the MPLL before the cache accesses it.
///
/// esp-idf asserts that the modem's PLL clock gate and its clock source agree. This port does not
/// assert, because a panic cannot print while the caches are suspended.
#[crate::ram]
pub(crate) fn pmu_sleep_finish() {
    // The eFuse controller reloads after a wake.
    while EFUSE::regs().status().read().state().bits() != 1 {}

    let psram = PSRAM_IN_SLEEP.load(Ordering::Relaxed);
    let mpll_mhz = MPLL_MHZ_BEFORE_SLEEP.load(Ordering::Relaxed);
    if mpll_mhz != 0 {
        // Like esp-idf's `pmu_sleep_finish`, this code powers the MPLL up, then sets its rate.
        clocks::set_mpll_power(true);
        clocks::set_mpll_frequency(mpll_mhz);
        if psram {
            set_psram_clock_for_sleep(false);
        }
    }
    if psram {
        hold_psram_pins(false);
    }

    // After a wake that did not retain the modem clock, the modem updates its block-ack bitmap and
    // start sequence registers only once the PLL clock reaches it. Like esp-idf's
    // `pmu_sleep_finish`, this code turns the modem's PLL clock gate on and off to work around
    // that.
    if HP_SYS_CLKRST::regs()
        .modem_conf()
        .read()
        .modem_pll_clk_en()
        .bit_is_clear()
    {
        let ref_160m = HP_SYS_CLKRST::regs().ref_160m_ctrl0();
        let ref_160m_on = ref_160m.read().clk_en().bit_is_set();
        if !ref_160m_on {
            ref_160m.modify(|_, w| w.clk_en().set_bit());
        }
        set_modem_pll_clock_gate(true);
        set_modem_pll_clock_gate(false);
        if !ref_160m_on {
            ref_160m.modify(|_, w| w.clk_en().clear_bit());
        }
        // Like esp-idf's `pmu_sleep_finish`, this code then powers the BBPLL down if the clock
        // tree does not use it.
        if !BBPLL_IN_CLOCK_TREE.load(Ordering::Relaxed) {
            disable_bbpll();
        }
    }

    crate::soc::disable_apm_filters();
}

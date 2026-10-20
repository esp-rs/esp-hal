use core::ops::Not;

use crate::{
    peripherals::{EFUSE, HP_APM, LP_AON, PMU},
    private::DropGuard,
    rtc_cntl::{
        Rtc,
        rtc::{
            HP_CALI_DRVB,
            HpAnalog,
            HpSysCntlReg,
            HpSysPower,
            LP_CALI_DBIAS,
            LpAnalog,
            LpSysPower,
        },
        sleep::{
            SleepKind,
            pmu_common::{SleepTimeConfig, request_sleep},
        },
    },
    soc::clocks::{self, AhbClkConfig, ClockTree, CpuClkConfig, CpuClkDivisor, HpRootClkConfig},
};

/// Configuration for controlling the behavior during sleep modes.
#[derive(Clone, Copy)]
// pmu_sleep_analog_config_t
pub struct AnalogSleepConfig {
    /// High-power system configuration.
    pub hp_sys: HpAnalog,
    /// Low-power system analog configuration.
    pub lp_sys_sleep: LpAnalog,
}

impl AnalogSleepConfig {
    fn defaults_deep_sleep() -> Self {
        Self {
            // PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT
            hp_sys: {
                let mut cfg = HpAnalog::default();

                cfg.bias.set_dcdc_ccm_enb(true);
                cfg.bias.set_dig_reg_dpcur_bias(1);
                cfg.bias.set_dig_reg_dsfmos(4);
                cfg.bias.set_dcm_vset(20);
                cfg.bias.set_dcm_mode(3);
                cfg.bias.set_discnnt_dig_rtc(false);
                cfg.bias.set_xpd_trx(false);
                cfg.bias.set_pd_cur(false);
                cfg.bias.set_bias_sleep(false);
                cfg.regulator0.set_xpd(false);

                cfg
            },
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();

                cfg.bias.set_dcdc_ccm_enb(true);
                cfg.bias.set_dig_reg_dpcur_bias(1);
                cfg.bias.set_dig_reg_dsfmos(4);
                cfg.bias.set_dcm_vset(0);
                cfg.bias.set_dcm_mode(3);
                cfg.bias.set_discnnt_dig_rtc(true);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(false);
                cfg.regulator0.set_dbias(3); // PMU_LP_DBIAS_SLEEP_0V7_DEFAULT
                cfg.regulator1.set_drv_b(0);

                cfg
            },
        }
    }

    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        let mut this = Self {
            // PMU_SLEEP_ANALOG_LSLP_CONFIG_DEFAULT
            hp_sys: {
                let mut cfg = HpAnalog::default();

                cfg.bias.set_dcdc_ccm_enb(true);
                cfg.bias.set_dig_reg_dpcur_bias(1);
                cfg.bias.set_dig_reg_dsfmos(4);
                cfg.bias.set_dcm_mode(3);
                cfg.bias.set_discnnt_dig_rtc(false);
                cfg.bias.set_xpd_trx(false);
                cfg.bias.set_xpd_bias(false);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_xpd(true);
                cfg.regulator0.set_dbias(0); // PMU_HP_DBIAS_LIGHTSLEEP_0V6_DEFAULT
                // PMU_HP_DRVB_LIGHTSLEEP_TOP_PD, PMU_HP_DRVB_LIGHTSLEEP_TOP_PU
                cfg.regulator1
                    .set_drv_b(if pd_flags.pd_top() { 23 } else { 25 });

                cfg
            },
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();

                cfg.bias.set_dcdc_ccm_enb(true);
                cfg.bias.set_dig_reg_dpcur_bias(1);
                cfg.bias.set_dig_reg_dsfmos(4);
                cfg.bias.set_dcm_vset(20);
                cfg.bias.set_dcm_mode(3);
                cfg.bias.set_discnnt_dig_rtc(false);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.regulator0.set_dbias(3); // PMU_LP_DBIAS_SLEEP_0V7_DEFAULT
                cfg.regulator1.set_drv_b(0);

                cfg
            },
        };

        // When XTAL or RC_FAST stays on during sleep, keep the regulators at their active-mode
        // calibration values.
        if !pd_flags.pd_xtal() {
            this.hp_sys.bias.set_xpd_trx(true);
            this.hp_sys.regulator1.set_drv_b(HP_CALI_DRVB);
            this.hp_sys.bias.set_pd_cur(false);
            this.hp_sys.bias.set_bias_sleep(false);

            this.lp_sys_sleep.bias.set_pd_cur(false);
            this.lp_sys_sleep.bias.set_bias_sleep(false);
            this.lp_sys_sleep.regulator0.set_dbias(LP_CALI_DBIAS);
            this.lp_sys_sleep.bias.set_dcm_vset(20);
        } else if !pd_flags.pd_rc_fast() {
            this.hp_sys.regulator1.set_drv_b(HP_CALI_DRVB);
            this.lp_sys_sleep.regulator0.set_dbias(LP_CALI_DBIAS);
        }

        this
    }

    fn apply(&self) {
        // pmu_sleep_analog_init

        // HP SLEEP (hp_sleep_*)
        PMU::regs().hp_sleep_bias().modify(|_, w| unsafe {
            w.hp_sleep_pd_cur().bit(self.hp_sys.bias.pd_cur());
            w.sleep().bit(self.hp_sys.bias.bias_sleep());
            w.hp_sleep_xpd_trx().bit(self.hp_sys.bias.xpd_trx());
            w.hp_sleep_discnnt_dig_rtc()
                .bit(self.hp_sys.bias.discnnt_dig_rtc());
            w.hp_sleep_dcm_mode().bits(self.hp_sys.bias.dcm_mode());
            w.hp_sleep_dcm_vset().bits(self.hp_sys.bias.dcm_vset())
        });
        PMU::regs().hp_sleep_hp_regulator0().modify(|_, w| unsafe {
            w.hp_sleep_hp_regulator_xpd()
                .bit(self.hp_sys.regulator0.xpd());
            w.hp_sleep_hp_regulator_dbias()
                .bits(self.hp_sys.regulator0.dbias())
        });
        PMU::regs().hp_sleep_hp_regulator1().modify(|_, w| unsafe {
            w.hp_sleep_hp_regulator_drv_b()
                .bits(self.hp_sys.regulator1.drv_b())
        });

        // LP SLEEP (lp_sleep_*)
        PMU::regs().lp_sleep_bias().modify(|_, w| unsafe {
            w.lp_sleep_pd_cur().bit(self.lp_sys_sleep.bias.pd_cur());
            w.sleep().bit(self.lp_sys_sleep.bias.bias_sleep());
            w.lp_sleep_discnnt_dig_rtc()
                .bit(self.lp_sys_sleep.bias.discnnt_dig_rtc());
            w.lp_sleep_dcm_mode()
                .bits(self.lp_sys_sleep.bias.dcm_mode());
            w.lp_sleep_dcm_vset()
                .bits(self.lp_sys_sleep.bias.dcm_vset())
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

/// Configuration for controlling the behavior of digital peripherals during
/// sleep modes.
#[derive(Clone, Copy)]
// pmu_sleep_digital_config_t
pub struct DigitalSleepConfig {
    /// High-power system control register configuration.
    pub syscntl: HpSysCntlReg,
    /// The functional clocks that stay enabled during sleep.
    pub sleep_icg_func: u32,
}

impl DigitalSleepConfig {
    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        // PMU_SLEEP_DIGITAL_LSLP_CONFIG_DEFAULT
        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_dig_pad_slp_sel(pd_flags.pd_top().not());
        syscntl.set_dig_pause_wdt(true);

        Self {
            syscntl,
            sleep_icg_func: 0,
        }
    }

    fn defaults_deep_sleep() -> Self {
        // PMU_SLEEP_DIGITAL_DSLP_CONFIG_DEFAULT
        let mut syscntl = HpSysCntlReg::default();
        syscntl.set_dig_pad_slp_sel(true);
        syscntl.set_dig_pause_wdt(true);

        Self {
            syscntl,
            sleep_icg_func: 0,
        }
    }

    fn apply(&self, deep: bool) {
        // pmu_sleep_digital_init
        PMU::regs().hp_sleep_hp_sys_cntl().modify(|_, w| {
            w.hp_sleep_dig_pause_wdt().bit(self.syscntl.dig_pause_wdt());
            if !deep {
                w.hp_sleep_dig_pad_slp_sel()
                    .bit(self.syscntl.dig_pad_slp_sel());
            }
            w
        });
        PMU::regs()
            .hp_sleep_sysclk()
            .modify(|_, w| w.hp_sleep_icg_sys_clock_en().bit(self.sleep_icg_func != 0));
        PMU::regs()
            .hp_sleep_icg_hp_func()
            .write(|w| unsafe { w.bits(self.sleep_icg_func) });
    }
}

/// Configuration for controlling the power settings of high-power and low-power
/// systems during sleep modes.
#[derive(Clone, Copy)]
// pmu_sleep_power_config_t
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
        // PMU_SLEEP_POWER_CONFIG_DEFAULT
        let mut hp_sys = HpSysPower::default();
        hp_sys
            .dig_power
            .set_vdd_flash_mode(if pd_flags.pd_vddsdio() { 1 } else { 3 });
        hp_sys.dig_power.set_wifi_pd_en(pd_flags.pd_modem());
        hp_sys.dig_power.set_cpu_pd_en(pd_flags.pd_cpu());
        hp_sys.dig_power.set_aon_pd_en(pd_flags.pd_hp_aon());
        hp_sys.dig_power.set_top_pd_en(pd_flags.pd_top());
        hp_sys.clk.set_i2c_iso_en(true);
        hp_sys.clk.set_i2c_retention(true);
        hp_sys.xtal.set_xpd_xtal(pd_flags.pd_xtal().not());

        let mut lp_sys_active = LpSysPower::default();
        lp_sys_active
            .clk_power
            .set_xpd_xtal32k(pd_flags.pd_xtal32k().not());
        lp_sys_active
            .clk_power
            .set_xpd_rc32k(pd_flags.pd_rc32k().not());
        lp_sys_active.clk_power.set_xpd_fosc(true);

        let mut lp_sys_sleep = LpSysPower::default();
        lp_sys_sleep.dig_power.set_vdd_io_mode(3);
        lp_sys_sleep
            .dig_power
            .set_peri_pd_en(pd_flags.pd_lp_periph());
        lp_sys_sleep
            .clk_power
            .set_xpd_xtal32k(pd_flags.pd_xtal32k().not());
        lp_sys_sleep
            .clk_power
            .set_xpd_rc32k(pd_flags.pd_rc32k().not());
        lp_sys_sleep
            .clk_power
            .set_xpd_fosc(pd_flags.pd_rc_fast().not());
        lp_sys_sleep.xtal.set_xpd_xtal(pd_flags.pd_xtal().not());

        Self {
            hp_sys,
            lp_sys_active,
            lp_sys_sleep,
        }
    }

    fn apply(&self, deep: bool) {
        // pmu_sleep_power_init

        // HP SLEEP (hp_sleep_*)
        PMU::regs()
            .hp_sleep_dig_power()
            .write(|w| unsafe { w.bits(self.hp_sys.dig_power.0) });
        PMU::regs()
            .hp_sleep_hp_ck_power()
            .write(|w| unsafe { w.bits(self.hp_sys.clk.0) });
        PMU::regs().hp_sleep_xtal().modify(|_, w| {
            w.hp_sleep_xpd_xtal().bit(self.hp_sys.xtal.xpd_xtal());
            w.hp_sleep_xpd_xtalx2().bit(self.hp_sys.xtal.xpd_xtalx2())
        });

        // pmu_ll_hp_set_memory_power_on_mask
        let mem_mask = if deep { 0 } else { 0xf };
        PMU::regs().power_pd_mem_mask().modify(|_, w| unsafe {
            w.pd_hp_mem0_mask().bits(mem_mask);
            w.pd_hp_mem1_mask().bits(mem_mask);
            w.pd_hp_mem2_mask().bits(mem_mask)
        });

        // LP ACTIVE (hp_sleep_lp_*)
        PMU::regs()
            .hp_sleep_lp_dig_power()
            .write(|w| unsafe { w.bits(self.lp_sys_active.dig_power.0) });
        PMU::regs()
            .hp_sleep_lp_ck_power()
            .write(|w| unsafe { w.bits(self.lp_sys_active.clk_power.0) });

        // LP SLEEP (lp_sleep_*)
        PMU::regs()
            .lp_sleep_lp_dig_power()
            .write(|w| unsafe { w.bits(self.lp_sys_sleep.dig_power.0) });
        PMU::regs()
            .lp_sleep_lp_ck_power()
            .write(|w| unsafe { w.bits(self.lp_sys_sleep.clk_power.0) });
        PMU::regs().lp_sleep_xtal().modify(|_, w| {
            w.lp_sleep_xpd_xtal().bit(self.lp_sys_sleep.xtal.xpd_xtal());
            w.lp_sleep_xpd_xtalx2()
                .bit(self.lp_sys_sleep.xtal.xpd_xtalx2())
        });
    }
}

/// Parameters for high-power system configurations during sleep modes.
#[derive(Clone, Copy)]
// pmu_hp_param_t
pub struct HpParam {
    /// Number of cycles to wait for the modem to wake up.
    pub modem_wakeup_wait_cycle: u32,
    /// Number of cycles to wait for the analog component stabilization.
    pub analog_wait_target_cycle: u16,
    /// Number of cycles to wait for the digital power supply to stabilize.
    pub digital_power_supply_wait_cycle: u16,
    /// Number of cycles to wait for the digital power-up sequence.
    pub digital_power_up_wait_cycle: u16,
    /// Number of cycles to wait for the PLL to stabilize.
    pub pll_stable_wait_cycle: u16,
    /// Minimum sleep time measured in slow clock cycles.
    pub min_slp_slow_clk_cycle: u8,
    /// Number of cycles to wait for all isolate signals to be ready.
    pub isolate_wait_cycle: u8,
    /// Number of cycles to wait for all reset signals to be ready.
    pub reset_wait_cycle: u8,
}

/// Parameters for low-power system configurations during sleep modes.
#[derive(Clone, Copy)]
// pmu_lp_param_t
pub struct LpParam {
    /// Number of cycles to wait for the digital power supply to stabilize.
    pub digital_power_supply_wait_cycle: u16,
    /// Minimum sleep time measured in slow clock cycles.
    pub min_slp_slow_clk_cycle: u8,
    /// Number of cycles to wait for the analog component stabilization.
    pub analog_wait_target_cycle: u8,
    /// Number of cycles to wait for the digital power-up sequence.
    pub digital_power_up_wait_cycle: u8,
    /// Number of cycles to wait for all isolate signals to be ready.
    pub isolate_wait_cycle: u8,
    /// Number of cycles to wait for all reset signals to be ready.
    pub reset_wait_cycle: u8,
}

/// Parameters for high-power and low-power system configurations during sleep
/// modes.
#[derive(Clone, Copy)]
// pmu_hp_lp_param_t
pub struct HpLpParam {
    /// Number of cycles to wait for the crystal oscillator to stabilize.
    ///
    /// The unit is the slow clock if XTAL stays on during sleep, else the fast clock.
    pub xtal_stable_wait_cycle: u16,
}

/// Configuration of parameters for sleep modes
#[derive(Clone, Copy)]
// pmu_sleep_param_config_t
pub struct ParamSleepConfig {
    /// Configuration of high-power system parameters.
    pub hp_sys: HpParam,
    /// Configuration of low-power system parameters.
    pub lp_sys: LpParam,
    /// Shared configuration parameters for high-power and low-power systems.
    pub hp_lp: HpLpParam,
}

impl ParamSleepConfig {
    const PMU_SLEEP_PARAM_CONFIG_DEFAULT: Self = Self {
        hp_sys: HpParam {
            min_slp_slow_clk_cycle: 10,
            analog_wait_target_cycle: 2419,
            digital_power_supply_wait_cycle: 32,
            digital_power_up_wait_cycle: 32,
            modem_wakeup_wait_cycle: 20700,
            pll_stable_wait_cycle: 2,
            isolate_wait_cycle: 0,
            reset_wait_cycle: 0,
        },
        lp_sys: LpParam {
            min_slp_slow_clk_cycle: 10,
            analog_wait_target_cycle: 23,
            digital_power_supply_wait_cycle: 32,
            digital_power_up_wait_cycle: 32,
            isolate_wait_cycle: 0,
            reset_wait_cycle: 0,
        },
        hp_lp: HpLpParam {
            xtal_stable_wait_cycle: 30,
        },
    };

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
            w.wait_xtl_stable().bits(self.hp_lp.xtal_stable_wait_cycle);
            w.wait_pll_stable().bits(self.hp_sys.pll_stable_wait_cycle)
        });
    }

    fn defaults(config: SleepTimeConfig, xpd_xtal: bool) -> Self {
        let mut param = Self::PMU_SLEEP_PARAM_CONFIG_DEFAULT;

        // pmu_sleep_param_config_default
        param.hp_sys.min_slp_slow_clk_cycle =
            config.us_to_slowclk(MachineConstants::HP_MIN_SLP_TIME_US) as u8;
        param.hp_sys.analog_wait_target_cycle =
            config.us_to_fastclk(MachineConstants::HP_ANALOG_WAIT_TIME_US) as u16;
        param.hp_sys.digital_power_supply_wait_cycle =
            config.us_to_fastclk(MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US) as u16;
        param.hp_sys.digital_power_up_wait_cycle =
            config.us_to_fastclk(MachineConstants::HP_POWER_UP_WAIT_TIME_US) as u16;
        param.hp_sys.pll_stable_wait_cycle =
            config.us_to_fastclk(MachineConstants::HP_PLL_WAIT_STABLE_TIME_US) as u16;
        param.hp_sys.isolate_wait_cycle =
            config.us_to_fastclk(MachineConstants::HP_ISOLATE_WAIT_TIME_US) as u8;
        param.hp_sys.reset_wait_cycle =
            config.us_to_fastclk(MachineConstants::HP_RESET_WAIT_TIME_US) as u8;

        param.lp_sys.min_slp_slow_clk_cycle =
            config.us_to_slowclk(MachineConstants::LP_MIN_SLP_TIME_US) as u8;
        param.lp_sys.analog_wait_target_cycle =
            config.us_to_slowclk(MachineConstants::LP_ANALOG_WAIT_TIME_US) as u8;
        param.lp_sys.digital_power_supply_wait_cycle =
            config.us_to_fastclk(MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US) as u16;
        param.lp_sys.digital_power_up_wait_cycle =
            config.us_to_fastclk(MachineConstants::LP_POWER_UP_WAIT_TIME_US) as u8;
        param.lp_sys.isolate_wait_cycle =
            config.us_to_fastclk(MachineConstants::LP_ISOLATE_WAIT_TIME_US) as u8;
        param.lp_sys.reset_wait_cycle =
            config.us_to_fastclk(MachineConstants::LP_RESET_WAIT_TIME_US) as u8;

        param.hp_lp.xtal_stable_wait_cycle = if xpd_xtal {
            config.us_to_slowclk(MachineConstants::LP_XTAL_WAIT_STABLE_TIME_US) as u16
        } else {
            config.us_to_fastclk(MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US) as u16
        };

        param
    }
}

impl SleepTimeConfig {
    pub(crate) const CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: u32 = 96;
    pub(crate) const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 9;

    pub(crate) fn pmu_sleep_calculate_hw_wait_time(&self, pd_flags: PowerDownFlags) -> u32 {
        // esp-hal always uses RC_FAST as the LP fast clock, so XTAL is never the RTC fast clock.

        // LP core hardware wait time, microsecond
        let lp_clk_switch_time_us = self.slowclk_to_us(MachineConstants::LP_CLK_SWITCH_CYCLE);
        let lp_clk_power_on_wait_time_us =
            self.slowclk_to_us(MachineConstants::LP_CLK_POWER_ON_WAIT_CYCLE);
        let lp_hw_wait_time_us = MachineConstants::LP_MIN_SLP_TIME_US
            + MachineConstants::LP_ANALOG_WAIT_TIME_US
            + lp_clk_power_on_wait_time_us
            + lp_clk_switch_time_us
            + MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::LP_POWER_UP_WAIT_TIME_US;

        // HP core hardware wait time, microsecond
        let hp_regdma_s2a_work_time_us = if pd_flags.pd_top() {
            MachineConstants::HP_REGDMA_S2A_WORK_TIME_PD_TOP_US
        } else {
            MachineConstants::HP_REGDMA_S2A_WORK_TIME_PU_TOP_US
        };
        let hp_digital_power_up_wait_time_us = MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::HP_POWER_UP_WAIT_TIME_US;
        let hp_control_wait_time_us =
            MachineConstants::HP_ISOLATE_WAIT_TIME_US + MachineConstants::HP_RESET_WAIT_TIME_US;
        let hp_regdma_wait_time_us = u32::max(
            MachineConstants::HP_REGDMA_S2M_WORK_TIME_US
                + MachineConstants::HP_REGDMA_M2A_WORK_TIME_US,
            hp_regdma_s2a_work_time_us,
        );
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
// pmu_sleep_config_t + deep sleep flag + pd flags
pub struct RtcSleepConfig {
    /// Deep Sleep flag
    pub deep: bool,
    /// Powers Down flags.
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
    /// Controls the power-down status of the modem power domain.
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

// Constants defined in `PMU_SLEEP_MC_DEFAULT()` (ESP32-H4 values)
struct MachineConstants;
impl MachineConstants {
    const LP_MIN_SLP_TIME_US: u32 = 450;
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
    const HP_REGDMA_S2M_WORK_TIME_US: u32 = 172;
    const HP_REGDMA_M2A_WORK_TIME_US: u32 = 278;
    const HP_REGDMA_S2A_WORK_TIME_PD_TOP_US: u32 = 0;
    const HP_REGDMA_S2A_WORK_TIME_PU_TOP_US: u32 = 390;
    const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const HP_PLL_WAIT_STABLE_TIME_US: u32 = 1;
}

impl RtcSleepConfig {
    /// Returns whether the device is in deep sleep mode.
    pub fn deep_slp(&self) -> bool {
        self.deep
    }

    /// Configures the device for deep sleep mode with ultra-low power settings.
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
            // Like esp-idf `esp_deep_sleep_start`. A wakeup source that needs the LP peripherals
            // keeps them powered.
            self.pd_flags.set_pd_top(true);
            self.pd_flags.set_pd_vddsdio(true);
            self.pd_flags.set_pd_modem(true);
            self.pd_flags.set_pd_hp_periph(true);
            self.pd_flags.set_pd_cpu(true);
            self.pd_flags.set_pd_hp_aon(true);
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_lp_periph(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
            self.pd_flags.set_pd_rc32k(true);
            self.pd_flags.set_pd_rc_fast(true);
        } else {
            // Light sleep: the digital domain (CPU, RAM, peripherals) stays powered and only
            // clock gated, so execution resumes in place. The sleep powers down the clock sources
            // that nothing uses while the core is gated. With XTAL and RC_FAST off, the analog
            // configuration also drops the regulators to their sleep voltages.
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
        }
    }

    /// Configures the wakeup and reject sources of the sleep.
    ///
    /// [`Self::enter_sleep`] requests the sleep after this call. The return value is a guard that
    /// restores what sleep entry changed for the sleep only, so the caller keeps it until the
    /// sleep ends.
    #[crate::ram]
    pub(crate) fn start_sleep(&self, wakeup_mask: u32, reject_mask: u32) -> impl Sized {
        // rtc_clk_cpu_freq_set_xtal_for_sleep: XTAL as the root clock, without dividers.
        let restore_clock_config = ClockTree::with(|clocks| {
            let old_root = clocks.hp_root_clk();
            let old_cpu = clocks.cpu_clk();
            let old_ahb = clocks.ahb_clk();

            clocks::configure_hp_root_clk(clocks, HpRootClkConfig::Xtal);
            clocks::configure_cpu_clk(clocks, CpuClkConfig::new(CpuClkDivisor::_0));
            clocks::configure_ahb_clk(clocks, AhbClkConfig::new(0));

            DropGuard::new((), move |_| {
                ClockTree::with(|clocks| {
                    crate::soc::clocks::reconfigure_pll(clocks);
                    // AHB_CLK may not exceed 32 MHz, so the dividers change before the root
                    // clock.
                    if let Some(old_ahb) = old_ahb {
                        clocks::configure_ahb_clk(clocks, old_ahb);
                    }
                    if let Some(old_cpu) = old_cpu {
                        clocks::configure_cpu_clk(clocks, old_cpu);
                    }
                    if let Some(old_root) = old_root {
                        clocks::configure_hp_root_clk(clocks, old_root);
                    }
                });
            })
        });

        // pmu_sleep_config_default + pmu_sleep_init.

        let power = PowerSleepConfig::defaults(self.pd_flags);
        power.apply(self.deep);

        // Needs to happen after rtc_clk_cpu_freq_set_xtal
        let config = if self.deep {
            SleepTimeConfig::deep_sleep()
        } else {
            SleepTimeConfig::light_sleep(self.pd_flags)
        };

        let mut param = ParamSleepConfig::defaults(config, power.hp_sys.xtal.xpd_xtal());

        if self.deep {
            const PMU_LP_ANALOG_WAIT_TARGET_TIME_DSLP_US: u32 = 500;
            param.lp_sys.analog_wait_target_cycle =
                config.us_to_slowclk(PMU_LP_ANALOG_WAIT_TARGET_TIME_DSLP_US) as u8;

            DigitalSleepConfig::defaults_deep_sleep().apply(true);
            AnalogSleepConfig::defaults_deep_sleep().apply();
        } else {
            DigitalSleepConfig::defaults_light_sleep(self.pd_flags).apply(false);
            AnalogSleepConfig::defaults_light_sleep(self.pd_flags).apply();
        }

        param.apply();

        // like esp-idf pmu_sleep_start()

        // lp_aon_hal_inform_wakeup_type - tells ROM which wakeup stub to run
        LP_AON::regs()
            .store8()
            .modify(|r, w| unsafe { w.bits(r.bits() & !0x01 | self.deep as u32) });

        // pmu_ll_hp_set_wakeup_enable
        PMU::regs()
            .slp_wakeup_cntl2()
            .write(|w| unsafe { w.bits(wakeup_mask) });

        // pmu_ll_hp_set_reject_enable
        PMU::regs().slp_wakeup_cntl1().modify(|_, w| unsafe {
            w.slp_reject_en().bit(reject_mask != 0);
            w.sleep_reject_ena().bits(reject_mask)
        });

        PMU::regs().int_clr().write(|w| {
            // pmu_ll_hp_clear_wakeup_intr_status
            w.soc_wakeup().clear_bit_by_one();
            // pmu_ll_hp_clear_reject_intr_status
            w.soc_sleep_reject().clear_bit_by_one()
        });

        // pmu_ll_hp_clear_reject_cause
        PMU::regs()
            .slp_wakeup_cntl4()
            .write(|w| w.slp_reject_cause_clr().bit(true));

        restore_clock_config
    }

    /// Requests the sleep.
    ///
    /// The caller waits for the result of the request.
    #[inline(always)]
    pub(crate) fn enter_sleep(&self) -> bool {
        request_sleep()
    }

    /// Cleans up after sleep.
    #[inline(always)]
    pub(crate) fn finish_sleep(&self) {
        // like esp-idf pmu_sleep_finish()

        // Wait until the eFuse controller is idle.
        while EFUSE::regs().status().read().state().bits() != 1 {}

        // apm_hal_enable_ctrl_filter_all(false), see `soc::pre_init`.
        HP_APM::regs().func_ctrl().write(|w| unsafe { w.bits(0) });

        // The post-wake hook of the GPIO driver releases the pads that the sleep armed. Only that
        // driver knows which pads it prepared.
    }
}

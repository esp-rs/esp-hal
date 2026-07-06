use core::ops::Not;

use crate::{
    clock::{RtcClock, calibrate_rtc_slow_clock, rtc_slow_cal_period},
    peripherals::{LP_AON, PMU},
    private::DropGuard,
    rtc_cntl::{
        Rtc,
        rtc::{HpAnalog, HpSysCntlReg, HpSysPower, LpAnalog, LpSysPower},
        sleep::WakeTriggers,
    },
    soc::clocks::{self, ClockTree, HpRootClkConfig, LpSlowClkConfig, TimgCalibrationClockConfig},
};

/// Configuration for controlling the behavior during sleep modes.
#[derive(Clone, Copy)]
// pmu_sleep_analog_config_t
pub struct AnalogSleepConfig {
    /// High-power system configuration.
    pub hp_sys: HpAnalog,
    // pub lp_sys_active: LpAnalog, // unused
    /// Low-power system analog configuration.
    pub lp_sys_sleep: LpAnalog,
}

impl AnalogSleepConfig {
    fn defaults_deep_sleep() -> Self {
        Self {
            // PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT
            hp_sys: {
                let mut cfg = HpAnalog::default();

                cfg.bias.set_pd_cur(true); // PMU_PD_CUR_SLEEP_DEFAULT
                cfg.bias.set_bias_sleep(true); // PMU_BIASSLP_SLEEP_DEFAULT
                cfg.regulator0.set_xpd(false);
                cfg.bias.set_dbg_atten(13); // PMU_DBG_HP_DEEPSLEEP

                cfg
            },
            // lp_sys_active: LpAnalog::default(),
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();

                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(13); // PMU_DBG_ATTEN_DEEPSLEEP_DEFAULT
                cfg.regulator0.set_dbias(23); // PMU_LP_DBIAS_DEEPSLEEP_0V7_DEFAULT

                cfg
            },
        }
    }

    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        let mut this = Self {
            // PMU_SLEEP_ANALOG_LSLP_CONFIG_DEFAULT
            hp_sys: {
                let mut cfg = HpAnalog::default();

                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(1); // PMU_DBG_ATTEN_LIGHTSLEEP_DEFAULT
                cfg.regulator0.set_dbias(0); // PMU_HP_DBIAS_LIGHTSLEEP_0V6_DEFAULT

                cfg
            },
            // lp_sys_active: LpAnalog::default(),
            lp_sys_sleep: {
                let mut cfg = LpAnalog::default();

                cfg.regulator1.set_drv_b(0);
                cfg.bias.set_pd_cur(true);
                cfg.bias.set_bias_sleep(true);
                cfg.regulator0.set_slp_xpd(false);
                cfg.regulator0.set_slp_dbias(0);
                cfg.regulator0.set_xpd(true);
                cfg.bias.set_dbg_atten(1); // PMU_DBG_ATTEN_LIGHTSLEEP_DEFAULT
                cfg.regulator0.set_dbias(15); // PMU_LP_DBIAS_LIGHTSLEEP_0V7_DEFAULT

                cfg
            },
        };

        // When XTAL or RC_FAST stays on during sleep, raise the regulator
        // voltage back to the active-mode calibration value.
        if !pd_flags.pd_xtal() || !pd_flags.pd_rc_fast() {
            this.hp_sys.bias.set_pd_cur(false); // PMU_PD_CUR_SLEEP_ON
            this.hp_sys.bias.set_bias_sleep(false); // PMU_BIASSLP_SLEEP_ON
            this.hp_sys.bias.set_dbg_atten(0);
            this.hp_sys.regulator0.set_dbias(26); // get_act_hp_dbias() ~ HP_CALI_DBIAS

            this.lp_sys_sleep.bias.set_pd_cur(false);
            this.lp_sys_sleep.bias.set_bias_sleep(false);
            this.lp_sys_sleep.bias.set_dbg_atten(0);
            this.lp_sys_sleep.regulator0.set_dbias(25); // get_act_lp_dbias() ~ LP_CALI_DBIAS
        }

        this
    }

    fn apply(&self) {
        // pmu_sleep_analog_init

        unsafe {
            // HP SLEEP (hp_sleep_*)
            PMU::regs().hp_sleep_bias().modify(|_, w| {
                // pmu_ll_hp_set_dbg_atten
                w.hp_sleep_dbg_atten().bits(self.hp_sys.bias.dbg_atten());
                // pmu_ll_hp_set_current_power_off
                w.hp_sleep_pd_cur().bit(self.hp_sys.bias.pd_cur());
                // pmu_ll_hp_set_bias_sleep_enable
                w.sleep().bit(self.hp_sys.bias.bias_sleep())
            });
            PMU::regs().hp_sleep_hp_regulator0().modify(|_, w| {
                // pmu_ll_hp_set_regulator_xpd
                w.hp_sleep_hp_regulator_xpd()
                    .bit(self.hp_sys.regulator0.xpd());
                // pmu_ll_hp_set_regulator_dbias
                w.hp_sleep_hp_regulator_dbias()
                    .bits(self.hp_sys.regulator0.dbias())
            });
            PMU::regs().hp_sleep_hp_regulator1().modify(|_, w| {
                // pmu_ll_hp_set_regulator_driver_bar
                w.hp_sleep_hp_regulator_drv_b()
                    .bits(self.hp_sys.regulator1.drv_b())
            });

            // LP SLEEP (lp_sleep_*)
            PMU::regs().lp_sleep_bias().modify(|_, w| {
                // pmu_ll_lp_set_dbg_atten
                w.lp_sleep_dbg_atten()
                    .bits(self.lp_sys_sleep.bias.dbg_atten());
                // pmu_ll_lp_set_current_power_off
                w.lp_sleep_pd_cur().bit(self.lp_sys_sleep.bias.pd_cur());
                // pmu_ll_lp_set_bias_sleep_enable
                w.sleep().bit(self.lp_sys_sleep.bias.bias_sleep())
            });

            PMU::regs().lp_sleep_lp_regulator0().modify(|_, w| {
                // pmu_ll_lp_set_regulator_slp_xpd
                w.lp_sleep_lp_regulator_slp_xpd()
                    .bit(self.lp_sys_sleep.regulator0.slp_xpd());
                // pmu_ll_lp_set_regulator_xpd
                w.lp_sleep_lp_regulator_xpd()
                    .bit(self.lp_sys_sleep.regulator0.xpd());
                // pmu_ll_lp_set_regulator_sleep_dbias
                w.lp_sleep_lp_regulator_slp_dbias()
                    .bits(self.lp_sys_sleep.regulator0.slp_dbias());
                // pmu_ll_lp_set_regulator_dbias
                w.lp_sleep_lp_regulator_dbias()
                    .bits(self.lp_sys_sleep.regulator0.dbias())
            });

            PMU::regs().lp_sleep_lp_regulator1().modify(|_, w| {
                // pmu_ll_lp_set_regulator_driver_bar
                w.lp_sleep_lp_regulator_drv_b()
                    .bits(self.lp_sys_sleep.regulator1.drv_b())
            });
        }
    }
}

/// Configuration for controlling the behavior of digital peripherals during
/// sleep modes.
#[derive(Clone, Copy)]
// pmu_sleep_digital_config_t
pub struct DigitalSleepConfig {
    /// High-power system control register configuration.
    pub syscntl: HpSysCntlReg,
}

impl DigitalSleepConfig {
    fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        Self {
            // PMU_SLEEP_DIGITAL_LSLP_CONFIG_DEFAULT
            syscntl: {
                let mut cfg = HpSysCntlReg::default();

                cfg.set_dig_pad_slp_sel(pd_flags.pd_top().not());

                cfg
            },
        }
    }

    fn apply(&self) {
        // pmu_sleep_digital_init
        PMU::regs().hp_sleep_hp_sys_cntl().modify(|_, w| {
            w.hp_sleep_dig_pad_slp_sel()
                .bit(self.syscntl.dig_pad_slp_sel())
        });
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
        self.hp_sys.dig_power.set_wifi_pd_en(pd_flags.pd_modem());
        self.hp_sys.dig_power.set_cpu_pd_en(pd_flags.pd_cpu());
        self.hp_sys.dig_power.set_aon_pd_en(pd_flags.pd_hp_aon());
        self.hp_sys.dig_power.set_top_pd_en(pd_flags.pd_top());

        self.hp_sys.clk.set_i2c_iso_en(true);
        self.hp_sys.clk.set_i2c_retention(true);

        self.hp_sys.xtal.set_xpd_xtal(pd_flags.pd_xtal().not());

        self.lp_sys_active.clk_power.set_xpd_xtal32k(true);
        self.lp_sys_active.clk_power.set_xpd_rc32k(true);
        self.lp_sys_active.clk_power.set_xpd_fosc(true);

        self.lp_sys_sleep
            .dig_power
            .set_peri_pd_en(pd_flags.pd_lp_periph());
        self.lp_sys_sleep.dig_power.set_mem_dslp(true);

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

    fn apply(&self) {
        // pmu_sleep_power_init

        // HP SLEEP (hp_sleep_*)
        PMU::regs()
            .hp_sleep_dig_power()
            .modify(|_, w| unsafe { w.bits(self.hp_sys.dig_power.0) });
        PMU::regs()
            .hp_sleep_hp_ck_power()
            .modify(|_, w| unsafe { w.bits(self.hp_sys.clk.0) });
        PMU::regs()
            .hp_sleep_xtal()
            .modify(|_, w| w.hp_sleep_xpd_xtal().bit(self.hp_sys.xtal.xpd_xtal()));

        // LP ACTIVE (hp_sleep_lp_*)
        PMU::regs()
            .hp_sleep_lp_dig_power()
            .modify(|_, w| unsafe { w.bits(self.lp_sys_active.dig_power.0) });
        PMU::regs()
            .hp_sleep_lp_ck_power()
            .modify(|_, w| unsafe { w.bits(self.lp_sys_active.clk_power.0) });

        // LP SLEEP (lp_sleep_*)
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

/// Parameters for high-power system configurations during sleep modes.
#[derive(Clone, Copy)]
// pmu_hp_param_t
pub struct HpParam {
    /// Number of cycles to wait for the modem to wake up.
    pub modem_wakeup_wait_cycle: u32,
    /// Number of cycles to wait for the analog component stabilization.
    pub analog_wait_target_cycle: u16,
    /// Number of cycles to wait for the digital power-down sequence.
    pub digital_power_down_wait_cycle: u16,
    /// Number of cycles to wait for the digital power supply to stabilize.
    pub digital_power_supply_wait_cycle: u16,
    /// Number of cycles to wait for the digital power-up sequence.
    pub digital_power_up_wait_cycle: u16,
    /// Number of cycles to wait for the PLL to stabilize.
    pub pll_stable_wait_cycle: u16,
    /// Number of cycles to wait for modifying the ICG control.
    pub modify_icg_cntl_wait_cycle: u8,
    /// Number of cycles to wait for switching the ICG control.
    pub switch_icg_cntl_wait_cycle: u8,
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
    /// Number of cycles to wait for the digital power-down sequence.
    pub digital_power_down_wait_cycle: u8,
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
    /// Union of two u16 variants
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

            digital_power_down_wait_cycle: 0,
            modify_icg_cntl_wait_cycle: 0,
            switch_icg_cntl_wait_cycle: 0,
            isolate_wait_cycle: 0,
            reset_wait_cycle: 0,
        },
        lp_sys: LpParam {
            min_slp_slow_clk_cycle: 10,
            analog_wait_target_cycle: 23,
            digital_power_supply_wait_cycle: 32,
            digital_power_up_wait_cycle: 32,

            digital_power_down_wait_cycle: 0,
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
            // pmu_ll_hp_set_min_sleep_cycle
            w.hp_min_slp_val().bits(self.hp_sys.min_slp_slow_clk_cycle);
            // pmu_ll_lp_set_min_sleep_cycle
            w.lp_min_slp_val().bits(self.lp_sys.min_slp_slow_clk_cycle)
        });

        PMU::regs().slp_wakeup_cntl7().modify(|_, w| unsafe {
            // pmu_ll_hp_set_analog_wait_target_cycle
            w.ana_wait_target()
                .bits(self.hp_sys.analog_wait_target_cycle)
        });

        PMU::regs().power_wait_timer0().modify(|_, w| unsafe {
            // pmu_ll_hp_set_digital_power_supply_wait_cycle
            w.dg_hp_pd_wait_timer()
                .bits(self.hp_sys.digital_power_supply_wait_cycle);
            // pmu_ll_hp_set_digital_power_up_wait_cycle
            w.dg_hp_powerup_timer()
                .bits(self.hp_sys.digital_power_up_wait_cycle)
        });

        PMU::regs().power_wait_timer1().modify(|_, w| unsafe {
            // pmu_ll_lp_set_digital_power_supply_wait_cycle
            w.dg_lp_pd_wait_timer()
                .bits(self.lp_sys.digital_power_supply_wait_cycle);
            // pmu_ll_lp_set_digital_power_up_wait_cycle
            w.dg_lp_powerup_timer()
                .bits(self.lp_sys.digital_power_up_wait_cycle)
        });

        // pmu_hal_hp/lp_set_control_ready_wait_cycle
        PMU::regs().power_wait_timer2().modify(|_, w| unsafe {
            w.dg_hp_iso_wait_timer()
                .bits(self.hp_sys.isolate_wait_cycle);
            w.dg_hp_rst_wait_timer().bits(self.hp_sys.reset_wait_cycle);
            w.dg_lp_iso_wait_timer()
                .bits(self.lp_sys.isolate_wait_cycle);
            w.dg_lp_rst_wait_timer().bits(self.lp_sys.reset_wait_cycle)
        });

        PMU::regs().slp_wakeup_cntl5().modify(|_, w| unsafe {
            // pmu_ll_lp_set_analog_wait_target_cycle
            w.lp_ana_wait_target()
                .bits(self.lp_sys.analog_wait_target_cycle);
            // pmu_ll_hp_set_modem_wakeup_wait_cycle
            w.modem_wait_target()
                .bits(self.hp_sys.modem_wakeup_wait_cycle)
        });
        PMU::regs().power_ck_wait_cntl().modify(|_, w| unsafe {
            // pmu_ll_hp_set_xtal_stable_wait_cycle
            w.wait_xtl_stable().bits(self.hp_lp.xtal_stable_wait_cycle);
            // pmu_ll_hp_set_pll_stable_wait_cycle
            w.wait_pll_stable().bits(self.hp_sys.pll_stable_wait_cycle)
        });
    }

    fn defaults(config: SleepTimeConfig, pd_flags: PowerDownFlags, pd_xtal: bool) -> Self {
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

        let hw_wait_time_us = config.pmu_sleep_calculate_hw_wait_time(pd_flags);

        let modem_wakeup_wait_time_us = (config.sleep_time_adjustment
            + MachineConstants::MODEM_STATE_SKIP_TIME_US
            + MachineConstants::HP_REGDMA_RF_ON_WORK_TIME_US)
            .saturating_sub(hw_wait_time_us);
        param.hp_sys.modem_wakeup_wait_cycle = config.us_to_fastclk(modem_wakeup_wait_time_us);

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

        // This looks different from esp-idf but it is the same:
        // Both `xtal_stable_wait_cycle` and `xtal_stable_wait_slow_clk_cycle` are
        // u16 variants of the same union
        param.hp_lp.xtal_stable_wait_cycle = if pd_xtal {
            config.us_to_slowclk(MachineConstants::LP_XTAL_WAIT_STABLE_TIME_US) as u16
        } else {
            config.us_to_fastclk(MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US) as u16
        };

        param
    }
}

#[derive(Clone, Copy)]
struct SleepTimeConfig {
    sleep_time_adjustment: u32,
    // TODO: esp-idf does some calibration here to determine slowclk_period
    slowclk_period: u32,
    fastclk_period: u32,
}

const CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: u32 = 160;

impl SleepTimeConfig {
    const RTC_CLK_CAL_FRACT: u32 = 19;

    fn rtc_clk_cal_fast() -> u32 {
        // Calibrate rtc fast clock, only PMU supported chips sleep process is needed.
        const FAST_CLK_SRC_CAL_CYCLES: u32 = 2048;
        RtcClock::calibrate(
            TimgCalibrationClockConfig::RcFastDivClk,
            FAST_CLK_SRC_CAL_CYCLES,
        )
    }

    fn rtc_clk_cal_slow() -> u32 {
        calibrate_rtc_slow_clock();
        rtc_slow_cal_period()
    }

    fn new() -> Self {
        Self {
            sleep_time_adjustment: 0,
            slowclk_period: Self::rtc_clk_cal_slow(),
            fastclk_period: Self::rtc_clk_cal_fast(),
        }
    }

    fn light_sleep(pd_flags: PowerDownFlags) -> Self {
        const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 56;

        let mut this = Self::new();

        let sw = LIGHT_SLEEP_TIME_OVERHEAD_US; // TODO
        let hw = this.pmu_sleep_calculate_hw_wait_time(pd_flags);

        this.sleep_time_adjustment = sw + hw;

        this
    }

    fn deep_sleep() -> Self {
        let mut this = Self::new();

        this.sleep_time_adjustment = 250 + 100 * 240 / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

        this
    }

    fn us_to_slowclk(&self, us: u32) -> u32 {
        (us << Self::RTC_CLK_CAL_FRACT) / self.slowclk_period
    }

    fn slowclk_to_us(&self, rtc_cycles: u32) -> u32 {
        (rtc_cycles * self.slowclk_period) >> Self::RTC_CLK_CAL_FRACT
    }

    fn us_to_fastclk(&self, us: u32) -> u32 {
        (us << Self::RTC_CLK_CAL_FRACT) / self.fastclk_period
    }

    fn pmu_sleep_calculate_hw_wait_time(&self, pd_flags: PowerDownFlags) -> u32 {
        // LP core hardware wait time, microsecond
        let lp_wakeup_wait_time_us = self.slowclk_to_us(MachineConstants::LP_WAKEUP_WAIT_CYCLE);
        let lp_clk_switch_time_us = self.slowclk_to_us(MachineConstants::LP_CLK_SWITCH_CYCLE);
        let lp_clk_power_on_wait_time_us = if pd_flags.pd_xtal() {
            MachineConstants::LP_XTAL_WAIT_STABLE_TIME_US
        } else {
            self.slowclk_to_us(MachineConstants::LP_CLK_POWER_ON_WAIT_CYCLE)
        };

        // ESP32-C61 adds control (isolate + reset) wait time to both domains.
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

        // HP core hardware wait time, microsecond
        let hp_digital_power_up_wait_time_us = MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::HP_POWER_UP_WAIT_TIME_US;
        let hp_control_wait_time_us =
            MachineConstants::HP_ISOLATE_WAIT_TIME_US + MachineConstants::HP_RESET_WAIT_TIME_US;
        let hp_regdma_wait_time_us = u32::max(
            MachineConstants::HP_REGDMA_S2M_WORK_TIME_US
                + MachineConstants::HP_REGDMA_M2A_WORK_TIME_US,
            MachineConstants::HP_REGDMA_S2A_WORK_TIME_US,
        );
        let hp_clock_wait_time_us = MachineConstants::HP_XTAL_WAIT_STABLE_TIME_US
            + MachineConstants::HP_PLL_WAIT_STABLE_TIME_US;

        let hp_hw_wait_time_us = MachineConstants::HP_ANALOG_WAIT_TIME_US
            + u32::max(
                hp_digital_power_up_wait_time_us + hp_regdma_wait_time_us,
                hp_clock_wait_time_us,
            )
            + hp_control_wait_time_us;

        #[rustfmt::skip] // ASCII art
        //  When the SOC wakeup (lp timer or GPIO wakeup) and Modem wakeup (Beacon wakeup) complete,
        // the soc wakeup will be delayed until the RF is turned on in Modem state.
        //
        //              modem wakeup                      TBTT, RF on by HW
        //                   |                                    |
        //                  \|/                                  \|/
        // PMU_HP_ACTIVE                                                                         /------
        // PMU_HP_MODEM                                           /------------//////////////////
        // PMU_HP_SLEEP   ----------------------//////////////////
        //                  /|\                /|\ /|\          /|\           /|\              /|\
        //                   |<- some hw wait ->|   |            |             |<- M2A switch ->|
        //                   |  slow cycles &   | soc wakeup     |                              |
        //                   |   FOSC cycles    |<- S2M switch ->|                              |
        //                   |                                                                  |
        //                   |<--      PMU guard time, also the maximum time for the SOC     -->|
        //                   |                           wake-up delay                          |
        //
        const CONFIG_ESP_RADIO_ENHANCED_LIGHT_SLEEP: bool = true;

        let (rf_on_protect_time_us, sync_time_us) = if CONFIG_ESP_RADIO_ENHANCED_LIGHT_SLEEP {
            (
                MachineConstants::HP_REGDMA_RF_ON_WORK_TIME_US,
                MachineConstants::HP_CLOCK_DOMAIN_SYNC_TIME_US,
            )
        } else {
            (0, 0)
        };

        lp_hw_wait_time_us + hp_hw_wait_time_us + sync_time_us + rf_on_protect_time_us
    }
}

/// Configuration for the RTC sleep behavior.
#[derive(Clone, Copy)]
// pmu_sleep_config_t + deep sleep flag + pd flags
pub struct RtcSleepConfig {
    /// Deep Sleep flag
    pub deep: bool,
    /// Power Down flags
    pub pd_flags: PowerDownFlags,
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        // from pmu_sleep_param_config_default
        // sleep flags will be applied by wakeup methods and apply

        Self {
            deep: false,
            pd_flags: PowerDownFlags(0),
        }
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy)]
    /// Power domains to be powered down during sleep
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
    pub u32, pd_cpu      , set_pd_cpu      : 4;
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
    /// Controls the power-down status of the 32kHz crystal oscillator.
    pub u32, pd_xtal32k  , set_pd_xtal32k  : 12;
    /// Controls the power-down status of the 32kHz RC oscillator.
    pub u32, pd_rc32k    , set_pd_rc32k    : 13;
    /// Controls the power-down status of the low-power peripheral domain.
    pub u32, pd_lp_periph, set_pd_lp_periph: 14;
}

impl PowerDownFlags {
    /// Checks whether all memory groups (G0, G1, G2, G3) are powered down.
    pub fn pd_mem(self) -> bool {
        self.pd_mem_g0() && self.pd_mem_g1() && self.pd_mem_g2() && self.pd_mem_g3()
    }

    /// Sets the power-down status for all memory groups (G0, G1, G2, G3) at
    /// once.
    pub fn set_pd_mem(&mut self, value: bool) {
        self.set_pd_mem_g0(value);
        self.set_pd_mem_g1(value);
        self.set_pd_mem_g2(value);
        self.set_pd_mem_g3(value);
    }
}

// Constants defined in `PMU_SLEEP_MC_DEFAULT()` (ESP32-C61 values)
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
    const HP_CLOCK_DOMAIN_SYNC_TIME_US: u32 = 35;
    const HP_SYSTEM_DFS_UP_WORK_TIME_US: u32 = 124;
    const HP_ANALOG_WAIT_TIME_US: u32 = 154;
    const HP_ISOLATE_WAIT_TIME_US: u32 = 1;
    const HP_RESET_WAIT_TIME_US: u32 = 1;
    const HP_POWER_SUPPLY_WAIT_TIME_US: u32 = 20;
    const HP_POWER_UP_WAIT_TIME_US: u32 = 2;
    const HP_REGDMA_S2M_WORK_TIME_US: u32 = 270;
    const HP_REGDMA_S2A_WORK_TIME_US: u32 = 800;
    const HP_REGDMA_M2A_WORK_TIME_US: u32 = 296;
    // Unused, but defined in esp-idf. May be needed later.
    // const HP_REGDMA_A2S_WORK_TIME_US: u32 = 586;
    const HP_REGDMA_RF_ON_WORK_TIME_US: u32 = 138;
    // Unused, but defined in esp-idf. May be needed later.
    // const HP_REGDMA_RF_OFF_WORK_TIME_US: u32 = 28;
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

    /// Configures the device for deep sleep mode with ultra-low power settings.
    pub fn deep() -> Self {
        // Set up for ultra-low power sleep. Wakeup sources may modify these settings.
        Self {
            deep: true,
            ..Self::default()
        }
    }

    pub(crate) fn is_deep_sleep(&self) -> bool {
        self.deep
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        Self::wake_io_reset();
    }

    fn wake_io_reset() {
        // loosely based on esp_deep_sleep_wakeup_io_reset
        //
        // ESP32-C5 does not yet expose the RTC IO pin abstraction in esp-hal, so
        // there are no EXT1 wakeup pins to reset here.
    }

    /// Finalize power-down flags, apply configuration based on the flags.
    pub(crate) fn apply(&mut self) {
        let lp_slow_uses_xtal32k = ClockTree::with(|clocks| {
            matches!(
                clocks::lp_slow_clk_config(clocks),
                Some(LpSlowClkConfig::Xtal32k)
            )
        });

        if self.deep {
            // force-disable certain power domains
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
            // Light sleep: the digital domain (CPU, RAM, peripherals) stays powered
            // and only clock-gated, so execution resumes in place. To cut power we
            // turn off the analog clock sources that nothing needs while the core is
            // gated. Powering down *both* XTAL and RC_FAST also makes the analog
            // config drop the HP regulator to the 0.6 V light-sleep voltage instead
            // of holding it at the active calibration voltage (see
            // `AnalogSleepConfig::defaults_light_sleep`), which is the dominant saving.
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
        }
    }

    /// Configures wakeup options and enters sleep.
    ///
    /// This function does not return if deep sleep is requested.
    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        const PMU_EXT0_WAKEUP_EN: u32 = 1 << 0;
        const PMU_EXT1_WAKEUP_EN: u32 = 1 << 1;
        const PMU_GPIO_WAKEUP_EN: u32 = 1 << 2;
        const PMU_LP_TIMER_WAKEUP_EN: u32 = 1 << 4;
        const PMU_WIFI_SOC_WAKEUP_EN: u32 = 1 << 5;
        const PMU_UART0_WAKEUP_EN: u32 = 1 << 6;
        const PMU_UART1_WAKEUP_EN: u32 = 1 << 7;
        const PMU_SDIO_WAKEUP_EN: u32 = 1 << 8;
        const PMU_BLE_SOC_WAKEUP_EN: u32 = 1 << 10;
        const PMU_LP_CORE_WAKEUP_EN: u32 = 1 << 11;
        const PMU_USB_WAKEUP_EN: u32 = 1 << 14;
        const MODEM_REJECT: u32 = 1 << 16;

        const RTC_SLEEP_REJECT_MASK: u32 = PMU_EXT0_WAKEUP_EN
            | PMU_EXT1_WAKEUP_EN
            | PMU_GPIO_WAKEUP_EN
            | PMU_LP_TIMER_WAKEUP_EN
            | PMU_WIFI_SOC_WAKEUP_EN
            | PMU_UART0_WAKEUP_EN
            | PMU_UART1_WAKEUP_EN
            | PMU_SDIO_WAKEUP_EN
            | PMU_BLE_SOC_WAKEUP_EN
            | PMU_LP_CORE_WAKEUP_EN
            | PMU_USB_WAKEUP_EN;

        let wakeup_mask = wakeup_triggers.as_u32();
        let reject_mask = if self.deep {
            0
        } else {
            // TODO: MODEM_REJECT if s_sleep_modem.wifi.phy_link != NULL
            let reject_mask = RTC_SLEEP_REJECT_MASK | MODEM_REJECT;
            wakeup_mask & reject_mask
        };

        let _restore_clock_config = ClockTree::with(|clocks| {
            let old_root = clocks.hp_root_clk();

            clocks::configure_hp_root_clk(clocks, HpRootClkConfig::Xtal);

            // Restore the old clock settings when we return
            DropGuard::new((), move |_| {
                ClockTree::with(|clocks| {
                    if let Some(old_root) = old_root {
                        clocks::configure_hp_root_clk(clocks, old_root);
                    }
                });
            })
        });

        // pmu_sleep_config_default + pmu_sleep_init.

        let power = PowerSleepConfig::defaults(self.pd_flags);
        power.apply();

        // Needs to happen after rtc_clk_cpu_freq_set_xtal
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

            AnalogSleepConfig::defaults_deep_sleep().apply();
        } else {
            AnalogSleepConfig::defaults_light_sleep(self.pd_flags).apply();
            DigitalSleepConfig::defaults_light_sleep(self.pd_flags).apply();
        }

        param.apply();

        // like esp-idf pmu_sleep_start()

        // lp_aon_hal_inform_wakeup_type - tells ROM which wakeup stub to run
        LP_AON::regs()
            .store9()
            .modify(|r, w| unsafe { w.bits(r.bits() & !0x01 | self.deep as u32) });

        // pmu_ll_hp_set_wakeup_enable
        PMU::regs()
            .slp_wakeup_cntl2()
            .write(|w| unsafe { w.bits(wakeup_mask) });

        // pmu_ll_hp_set_reject_enable
        PMU::regs().slp_wakeup_cntl1().modify(|_, w| unsafe {
            w.slp_reject_en().bit(true);
            w.sleep_reject_ena().bits(reject_mask)
        });

        // pmu_ll_hp_clear_reject_cause
        PMU::regs()
            .slp_wakeup_cntl4()
            .write(|w| w.slp_reject_cause_clr().bit(true));

        PMU::regs().int_clr().write(|w| {
            // pmu_ll_hp_clear_sw_intr_status
            w.sw().clear_bit_by_one();
            // pmu_ll_hp_clear_reject_intr_status
            w.soc_sleep_reject().clear_bit_by_one();
            // pmu_ll_hp_clear_wakeup_intr_status
            w.soc_wakeup().clear_bit_by_one()
        });

        // misc_modules_sleep_prepare

        // Start entry into sleep mode

        // pmu_ll_hp_set_sleep_enable
        PMU::regs()
            .slp_wakeup_cntl0()
            .write(|w| w.sleep_req().bit(true));

        // In pd_cpu lightsleep and deepsleep mode, we never get here
        loop {
            let int_raw = PMU::regs().int_raw().read();
            if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                break;
            }
        }
    }

    /// Cleans up after sleep
    pub(crate) fn finish_sleep(&self) {
        // like esp-idf pmu_sleep_finish()
        // In "pd_cpu lightsleep" and "deepsleep" modes we never get here

        // esp-idf returns if the sleep was rejected, we do nothing
        // pmu_ll_hp_is_sleep_reject(PMU_instance()->hal->dev)

        Self::wake_io_reset();
    }
}

use core::ops::Not;

use crate::{
    peripherals::{LP_AON, PMU},
    private::DropGuard,
    rtc_cntl::{
        Rtc,
        rtc::{HpSysCntlReg, HpSysPower, LpSysPower},
        sleep::{Ext1WakeupSource, WakeTriggers, pmu_common::SleepTimeConfig},
    },
    soc::clocks::{self, ClockTree, CpuClkConfig, HpRootClkConfig, LpSlowClkConfig},
};

/// Configuration for controlling the behavior during sleep modes.
#[derive(Clone, Copy)]
// Note: from all the fields defined in ESP-IDF's `pmu_sleep_analog_config_t`
// the only one documented in Hardware Technical Reference Manual is `hp_sys.analog.xpd`
// (corresponds to `PMU_HP_ACTIVE_HP_REGULATOR_XPD` in the Manual). Therefore only this
// field is stored as bool without any nested structures.
pub struct AnalogSleepConfig {
    /// High-power system configuration.
    pub regulator_xpd: bool,
}

impl AnalogSleepConfig {
    fn defaults_deep_sleep() -> Self {
        Self {
            // PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT
            regulator_xpd: false,
        }
    }

    fn defaults_light_sleep() -> Self {
        Self {
            // PMU_SLEEP_ANALOG_LSLP_CONFIG_DEFAULT
            regulator_xpd: true,
        }
    }

    fn apply(&self) {
        // based on pmu_sleep_analog_init, with undocumented registers omitted
        PMU::regs().hp_sleep_hp_regulator0().modify(|_, w| {
            w.hp_sleep_hp_regulator_xpd() // pmu_ll_hp_set_regulator_xpd
                .bit(self.regulator_xpd)
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
    /// ICG function control flags.
    pub icg_func: u32,
    /// Indicates whether deep sleep mode is requested.
    pub deep_sleep: bool,
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
            icg_func: 0xffff_ffff, // TODO: ESP-IDF determines this using get_sleep_clock_icg_flags
            deep_sleep: false,
        }
    }

    fn defaults_deep_sleep() -> Self {
        Self {
            // PMU_SLEEP_DIGITAL_DSLP_CONFIG_DEFAULT
            syscntl: HpSysCntlReg::default(),
            icg_func: 0,
            deep_sleep: true,
        }
    }

    fn apply(&self) {
        // pmu_sleep_digital_init
        PMU::regs().hp_sleep_sysclk().modify(|_, w| {
            w.hp_sleep_icg_sys_clock_en().bit(self.icg_func != 0) // pmu_ll_hp_set_icg_sysclk_enable
        });
        PMU::regs().hp_sleep_icg_hp_func().modify(|_, w| unsafe {
            w.hp_sleep_dig_icg_func_en().bits(self.icg_func) // pmu_ll_hp_set_icg_func
        });
        if !self.deep_sleep {
            PMU::regs().hp_sleep_hp_sys_cntl().modify(|_, w| {
                w.hp_sleep_dig_pad_slp_sel()
                    .bit(self.syscntl.dig_pad_slp_sel()) // pmu_ll_hp_set_dig_pad_slp_sel
            });
        }
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
        self.hp_sys.dig_power.set_modem_pd_en(pd_flags.pd_modem());
        self.hp_sys.dig_power.set_cpu_pd_en(pd_flags.pd_cpu());
        self.hp_sys.dig_power.set_top_pd_en(pd_flags.pd_top());

        self.hp_sys.clk.set_xpd_bbpll(false);

        self.hp_sys.xtal.set_xpd_xtal(pd_flags.pd_xtal().not());

        self.lp_sys_active.clk_power.set_xpd_xtal32k(true);
        self.lp_sys_active.clk_power.set_xpd_fosc(true);

        self.lp_sys_sleep.dig_power.set_mem_dslp(true);
        self.lp_sys_sleep.dig_power.set_vddbat_mode(0);
        self.lp_sys_sleep.dig_power.set_bod_source_sel(true);

        self.lp_sys_sleep
            .clk_power
            .set_xpd_xtal32k(pd_flags.pd_xtal32k().not());
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
            analog_wait_target_cycle: 1700,
            digital_power_supply_wait_cycle: 32,
            digital_power_up_wait_cycle: 32,
            modem_wakeup_wait_cycle: 0,
            pll_stable_wait_cycle: 2,

            digital_power_down_wait_cycle: 0,
            modify_icg_cntl_wait_cycle: 0,
            switch_icg_cntl_wait_cycle: 0,
        },
        lp_sys: LpParam {
            min_slp_slow_clk_cycle: 10,
            analog_wait_target_cycle: 15,
            digital_power_supply_wait_cycle: 32,
            digital_power_up_wait_cycle: 32,

            digital_power_down_wait_cycle: 0,
        },
        hp_lp: HpLpParam {
            xtal_stable_wait_cycle: 30,
        },
    };

    fn apply(&self) {
        // pmu_sleep_param_init
        unsafe {
            PMU::regs().slp_wakeup_cntl3().modify(|_, w| {
                // pmu_ll_hp_set_min_sleep_cycle
                w.hp_min_slp_val().bits(self.hp_sys.min_slp_slow_clk_cycle);
                // pmu_ll_lp_set_min_sleep_cycle
                w.lp_min_slp_val().bits(self.lp_sys.min_slp_slow_clk_cycle)
            });

            PMU::regs().slp_wakeup_cntl7().modify(|_, w| {
                w.ana_wait_target() // pmu_ll_hp_set_analog_wait_target_cycle
                    .bits(self.hp_sys.analog_wait_target_cycle)
            });

            PMU::regs().slp_wakeup_cntl5().modify(|_, w| {
                w.lp_ana_wait_target() // pmu_ll_lp_set_analog_wait_target_cycle
                    .bits(self.lp_sys.analog_wait_target_cycle)
            });

            PMU::regs().power_wait_timer0().modify(|_, w| {
                // pmu_ll_hp_set_digital_power_supply_wait_cycle
                w.dg_hp_wait_timer()
                    .bits(self.hp_sys.digital_power_supply_wait_cycle);
                // pmu_ll_hp_set_digital_power_up_wait_cycle
                w.dg_hp_powerup_timer()
                    .bits(self.hp_sys.digital_power_up_wait_cycle)
            });

            PMU::regs().power_wait_timer1().modify(|_, w| {
                // pmu_ll_lp_set_digital_power_supply_wait_cycle
                w.dg_lp_wait_timer()
                    .bits(self.lp_sys.digital_power_supply_wait_cycle);
                // pmu_ll_lp_set_digital_power_up_wait_cycle
                w.dg_lp_powerup_timer()
                    .bits(self.lp_sys.digital_power_up_wait_cycle)
            });

            PMU::regs().power_ck_wait_cntl().modify(|_, w| {
                // pmu_ll_set_xtal_stable_wait_cycle
                w.wait_xtl_stable().bits(self.hp_lp.xtal_stable_wait_cycle);
                // pmu_ll_set_pll_stable_wait_cycle
                w.wait_pll_stable().bits(self.hp_sys.pll_stable_wait_cycle)
            });
        }
    }

    fn defaults(config: SleepTimeConfig, pd_xtal: bool) -> Self {
        let mut param = Self::PMU_SLEEP_PARAM_CONFIG_DEFAULT;

        // pmu_sleep_param_config_default
        param.hp_sys.min_slp_slow_clk_cycle =
            config.us_to_slowclk(machine_constants::HP_MIN_SLP_TIME_US) as u8;
        param.hp_sys.analog_wait_target_cycle =
            config.us_to_fastclk(machine_constants::HP_ANALOG_WAIT_TIME_US) as u16;
        param.hp_sys.digital_power_supply_wait_cycle =
            config.us_to_fastclk(machine_constants::HP_POWER_SUPPLY_WAIT_TIME_US) as u16;
        param.hp_sys.digital_power_up_wait_cycle =
            config.us_to_fastclk(machine_constants::HP_POWER_UP_WAIT_TIME_US) as u16;
        param.hp_sys.pll_stable_wait_cycle =
            config.us_to_fastclk(machine_constants::HP_PLL_WAIT_STABLE_TIME_US) as u16;

        param.lp_sys.min_slp_slow_clk_cycle =
            config.us_to_slowclk(machine_constants::LP_MIN_SLP_TIME_US) as u8;
        param.lp_sys.analog_wait_target_cycle =
            config.us_to_slowclk(machine_constants::LP_ANALOG_WAIT_TIME_US) as u8;
        param.lp_sys.digital_power_supply_wait_cycle =
            config.us_to_fastclk(machine_constants::LP_POWER_SUPPLY_WAIT_TIME_US) as u16;
        param.lp_sys.digital_power_up_wait_cycle =
            config.us_to_fastclk(machine_constants::LP_POWER_UP_WAIT_TIME_US) as u8;

        // This looks different from esp-idf but it is the same:
        // Both `xtal_stable_wait_cycle` and `xtal_stable_wait_slow_clk_cycle` are
        // u16 variants of the same union
        param.hp_lp.xtal_stable_wait_cycle = if pd_xtal {
            config.us_to_slowclk(machine_constants::LP_XTAL_WAIT_STABLE_TIME_US) as u16
        } else {
            config.us_to_fastclk(machine_constants::HP_XTAL_WAIT_STABLE_TIME_US) as u16
        };

        param
    }
}

impl SleepTimeConfig {
    pub(crate) const CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: u32 = 96;
    pub(crate) const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 9;

    pub(crate) fn pmu_sleep_calculate_hw_wait_time(&self, pd_flags: PowerDownFlags) -> u32 {
        let lp_clk_switch_time_us = self.slowclk_to_us(machine_constants::LP_CLK_SWITCH_CYCLE);
        let lp_clk_power_on_wait_time_us = if pd_flags.pd_xtal() {
            machine_constants::LP_XTAL_WAIT_STABLE_TIME_US
        } else {
            self.slowclk_to_us(machine_constants::LP_CLK_POWER_ON_WAIT_CYCLE)
        };

        let lp_hw_wait_time_us = machine_constants::LP_MIN_SLP_TIME_US
            + machine_constants::LP_ANALOG_WAIT_TIME_US
            + lp_clk_power_on_wait_time_us
            + lp_clk_switch_time_us
            + machine_constants::LP_POWER_SUPPLY_WAIT_TIME_US
            + machine_constants::LP_POWER_UP_WAIT_TIME_US;

        let hp_digital_power_up_wait_time_us = machine_constants::HP_POWER_SUPPLY_WAIT_TIME_US
            + machine_constants::HP_POWER_UP_WAIT_TIME_US;
        let hp_regdma_wait_time_us = if pd_flags.pd_top() {
            machine_constants::HP_REGDMA_S2A_WORK_TIME_PD_TOP_US
        } else {
            machine_constants::HP_REGDMA_S2A_WORK_TIME_PU_TOP_US
        };
        let hp_clock_wait_time_us = machine_constants::HP_XTAL_WAIT_STABLE_TIME_US
            + machine_constants::HP_PLL_WAIT_STABLE_TIME_US;

        let hp_hw_wait_time_us = machine_constants::HP_ANALOG_WAIT_TIME_US
            + hp_digital_power_up_wait_time_us
            + hp_regdma_wait_time_us
            + hp_clock_wait_time_us;

        lp_hw_wait_time_us + hp_hw_wait_time_us
    }
}

/// Configuration for the RTC sleep behavior.
#[derive(Clone, Copy)]
pub struct RtcSleepConfig {
    /// Deep Sleep flag
    pub deep: bool,
    /// Power Down flags
    pub pd_flags: PowerDownFlags,
    /// Indicates whether the top power domain should remain enabled during sleep.
    need_pd_top: bool,
}

impl Default for RtcSleepConfig {
    fn default() -> Self {
        // from pmu_sleep_param_config_default
        // sleep flags will be applied by wakeup methods and apply

        Self {
            deep: false,
            pd_flags: PowerDownFlags(0),
            need_pd_top: false,
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
    /// Controls the power-down status of the CPU power domain.
    pub u32, pd_cpu      , set_pd_cpu      : 3;
    /// Controls the power-down status of the crystal oscillator.
    pub u32, pd_xtal     , set_pd_xtal     : 4;
    /// Controls the power-down status of the fast RC oscillator.
    pub u32, pd_rc_fast  , set_pd_rc_fast  : 5;
    /// Controls the power-down status of the 32kHz crystal oscillator.
    pub u32, pd_xtal32k  , set_pd_xtal32k  : 6;
}

/// Constants defined in `PMU_SLEEP_MC_DEFAULT()`
mod machine_constants {
    pub const LP_MIN_SLP_TIME_US: u32 = 450;
    pub const LP_ANALOG_WAIT_TIME_US: u32 = 154;
    pub const LP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    pub const LP_CLK_SWITCH_CYCLE: u32 = 1;
    pub const LP_CLK_POWER_ON_WAIT_CYCLE: u32 = 1;
    pub const LP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    pub const LP_POWER_UP_WAIT_TIME_US: u32 = 2;

    pub const HP_MIN_SLP_TIME_US: u32 = 450;
    pub const HP_ANALOG_WAIT_TIME_US: u32 = 154;
    pub const HP_POWER_SUPPLY_WAIT_TIME_US: u32 = 2;
    pub const HP_POWER_UP_WAIT_TIME_US: u32 = 2;
    pub const HP_REGDMA_S2A_WORK_TIME_PD_TOP_US: u32 = 480;
    pub const HP_REGDMA_S2A_WORK_TIME_PU_TOP_US: u32 = 390;
    pub const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    pub const HP_PLL_WAIT_STABLE_TIME_US: u32 = 50;
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
        Ext1WakeupSource::wake_io_reset();
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
            self.pd_flags.set_pd_top(self.need_pd_top.not());
            self.pd_flags.set_pd_vddsdio(true);
            self.pd_flags.set_pd_modem(true);
            self.pd_flags.set_pd_cpu(true);
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
        } else {
            // Light sleep: the digital and top domains stay powered (so execution
            // resumes in place; the top domain must also stay on for any EXT1/GPIO
            // wakeup). Power down the analog clock sources nothing needs while the
            // core is clock-gated to cut power. Unlike the C5/C6 family, H2's
            // light-sleep analog config does not lower the HP voltage, so this saves
            // the oscillator current but not regulator power.
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags.set_pd_xtal32k(!lp_slow_uses_xtal32k);
        }
    }

    /// Configures wakeup options and enters sleep.
    ///
    /// This function does not return if deep sleep is requested.
    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        const PMU_EXT1_WAKEUP_EN: u32 = 1 << 1;
        const PMU_GPIO_WAKEUP_EN: u32 = 1 << 2;
        const PMU_LP_TIMER_WAKEUP_EN: u32 = 1 << 4;
        const PMU_UART0_WAKEUP_EN: u32 = 1 << 6;
        const PMU_UART1_WAKEUP_EN: u32 = 1 << 7;
        const PMU_BLE_SOC_WAKEUP_EN: u32 = 1 << 10;

        const RTC_SLEEP_REJECT_MASK: u32 = PMU_EXT1_WAKEUP_EN
            | PMU_GPIO_WAKEUP_EN
            | PMU_LP_TIMER_WAKEUP_EN
            | PMU_UART0_WAKEUP_EN
            | PMU_UART1_WAKEUP_EN
            | PMU_BLE_SOC_WAKEUP_EN;

        let wakeup_mask = wakeup_triggers.as_u32();
        let reject_mask = if self.deep {
            0
        } else {
            let reject_mask = RTC_SLEEP_REJECT_MASK;
            wakeup_mask & reject_mask
        };

        let _restore_clock_config = ClockTree::with(|clocks| {
            let old_hp_root_clk = clocks.hp_root_clk();
            let old_cpu_divider = clocks.cpu_clk();

            clocks::configure_hp_root_clk(clocks, HpRootClkConfig::Xtal);
            clocks::configure_cpu_clk(clocks, CpuClkConfig::new(0));

            // Restore the old clock settings when we return
            DropGuard::new((), move |_| {
                ClockTree::with(|clocks| {
                    if let Some(old_hp_root_clk) = old_hp_root_clk {
                        clocks::configure_hp_root_clk(clocks, old_hp_root_clk);
                    }
                    if let Some(old_cpu_divider) = old_cpu_divider {
                        clocks::configure_cpu_clk(clocks, old_cpu_divider);
                    }
                });
            })
        });

        // misc_modules_sleep_prepare

        // pmu_sleep_config_default + pmu_sleep_init

        let power = PowerSleepConfig::defaults(self.pd_flags);
        power.apply();

        // Needs to happen after rtc_clk_cpu_freq_set_xtal
        let config = if self.deep {
            SleepTimeConfig::deep_sleep()
        } else {
            SleepTimeConfig::light_sleep(self.pd_flags)
        };

        let param = ParamSleepConfig::defaults(config, power.hp_sys.xtal.xpd_xtal());

        if self.deep {
            DigitalSleepConfig::defaults_deep_sleep().apply();
            AnalogSleepConfig::defaults_deep_sleep().apply();
        } else {
            DigitalSleepConfig::defaults_light_sleep(self.pd_flags).apply();
            AnalogSleepConfig::defaults_light_sleep().apply();
        }

        param.apply();

        // pmu_sleep_start

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

        // Start entry into sleep mode

        // pmu_ll_hp_set_sleep_enable
        PMU::regs()
            .slp_wakeup_cntl0()
            .write(|w| w.sleep_req().bit(true));

        loop {
            let int_raw = PMU::regs().int_raw().read();
            if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                break;
            }
        }
    }

    /// Cleans up after sleep
    pub(crate) fn finish_sleep(&self) {
        Self::wake_io_reset();
    }
}

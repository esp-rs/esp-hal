use core::ops::Not;

use crate::{
    clock::Clock,
    efuse::Efuse,
    gpio::{Pins, RtcFunction},
    peripherals::Peripherals,
    rtc_cntl::{
        rtc::{
            rtc_clk_cpu_freq_set_xtal,
            HpAnalog,
            HpSysCntlReg,
            HpSysPower,
            LpAnalog,
            LpSysPower,
            RtcCalSel,
            SavedClockConfig,
        },
        sleep::{
            Ext1WakeupSource,
            TimerWakeupSource,
            WakeFromLpCoreWakeupSource,
            WakeSource,
            WakeTriggers,
            WakeupLevel,
        },
        Rtc,
        RtcClock,
    },
};

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);

        let lp_timer = unsafe { &*esp32c6::LP_TIMER::ptr() };
        let clock_freq = RtcClock::get_slow_freq();
        // TODO: maybe add sleep time adjustment like idf
        // TODO: maybe add check to prevent overflow?
        let clock_hz = clock_freq.frequency().to_Hz() as u64;
        let ticks = self.duration.as_micros() as u64 * clock_hz / 1_000_000u64;
        // "alarm" time in slow rtc ticks
        let now = rtc.time_since_boot_raw();
        let time_in_ticks = now + ticks;
        unsafe {
            lp_timer.tar0_high().write(|w| {
                w.main_timer_tar_high0()
                    .bits(((time_in_ticks >> 32) & 0xffff) as u16)
            });
            lp_timer.tar0_low().write(|w| {
                w.main_timer_tar_low0()
                    .bits((time_in_ticks & 0xffffffff) as u32)
            });
            lp_timer
                .int_clr()
                .write(|w| w.soc_wakeup().clear_bit_by_one());
            lp_timer
                .tar0_high()
                .modify(|_, w| w.main_timer_tar_en0().set_bit());
        }
    }
}

impl Ext1WakeupSource<'_, '_> {
    /// Returns the currently configured wakeup pins.
    fn wakeup_pins() -> u8 {
        unsafe { lp_aon().ext_wakeup_cntl().read().ext_wakeup_sel().bits() }
    }

    fn wake_io_reset(pins: &mut Pins) {
        use crate::gpio::RtcPin;

        fn uninit_pin(pin: &mut impl RtcPin, wakeup_pins: u8) {
            if wakeup_pins & (1 << pin.number()) != 0 {
                pin.rtcio_pad_hold(false);
                pin.rtc_set_config(false, false, RtcFunction::Rtc);
            }
        }

        let wakeup_pins = Ext1WakeupSource::wakeup_pins();
        uninit_pin(&mut pins.gpio0, wakeup_pins);
        uninit_pin(&mut pins.gpio1, wakeup_pins);
        uninit_pin(&mut pins.gpio2, wakeup_pins);
        uninit_pin(&mut pins.gpio3, wakeup_pins);
        uninit_pin(&mut pins.gpio4, wakeup_pins);
        uninit_pin(&mut pins.gpio5, wakeup_pins);
        uninit_pin(&mut pins.gpio6, wakeup_pins);
        uninit_pin(&mut pins.gpio7, wakeup_pins);
    }
}

impl WakeSource for Ext1WakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        // We don't have to keep the LP domain powered if we hold the wakeup pin states.
        triggers.set_ext1(true);

        // set pins to RTC function
        let mut pins = self.pins.borrow_mut();
        let mut pin_mask = 0u8;
        let mut level_mask = 0u8;
        for (pin, level) in pins.iter_mut() {
            pin_mask |= 1 << pin.number();
            level_mask |= match level {
                WakeupLevel::High => 1 << pin.number(),
                WakeupLevel::Low => 0,
            };

            pin.rtc_set_config(true, true, RtcFunction::Rtc);
            pin.rtcio_pad_hold(true);
        }

        unsafe {
            // clear previous wakeup status
            lp_aon()
                .ext_wakeup_cntl()
                .modify(|_, w| w.ext_wakeup_status_clr().set_bit());

            // set pin + level register fields
            lp_aon().ext_wakeup_cntl().modify(|r, w| {
                w.ext_wakeup_sel()
                    .bits(r.ext_wakeup_sel().bits() | pin_mask)
                    .ext_wakeup_lv()
                    .bits(r.ext_wakeup_lv().bits() & !pin_mask | level_mask)
            });
        }
    }
}

impl Drop for Ext1WakeupSource<'_, '_> {
    fn drop(&mut self) {
        // should we have saved the pin configuration first?
        // set pin back to IO_MUX (input_enable and func have no effect when pin is sent
        // to IO_MUX)
        let mut pins = self.pins.borrow_mut();
        for (pin, _level) in pins.iter_mut() {
            pin.rtc_set_config(true, false, RtcFunction::Rtc);
        }
    }
}

impl WakeSource for WakeFromLpCoreWakeupSource {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_lp_core(true);
    }
}

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

                cfg.bias.set_pd_cur(false);
                cfg.bias.set_bias_sleep(false);
                cfg.regulator0.set_xpd(false);
                cfg.bias.set_dbg_atten(0);

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
                cfg.bias.set_dbg_atten(12);
                cfg.regulator0.set_dbias(23); // 0.7V

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
                cfg.bias.set_dbg_atten(0);
                cfg.regulator0.set_dbias(1); // 0.6V

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
                cfg.bias.set_dbg_atten(0);
                cfg.regulator0.set_dbias(12); // 0.7V

                cfg
            },
        };

        if !pd_flags.pd_xtal() {
            this.hp_sys.bias.set_pd_cur(false);
            this.hp_sys.bias.set_bias_sleep(false);
            this.hp_sys.regulator0.set_dbias(25);

            this.lp_sys_sleep.bias.set_pd_cur(false);
            this.lp_sys_sleep.bias.set_bias_sleep(false);
            this.lp_sys_sleep.regulator0.set_dbias(26);
        }

        this
    }

    fn apply(&self) {
        // pmu_sleep_analog_init

        unsafe {
            // HP SLEEP (hp_sleep_*)
            pmu().hp_sleep_bias().modify(|_, w| {
                w.hp_sleep_dbg_atten() // pmu_ll_hp_set_dbg_atten
                    .bits(self.hp_sys.bias.dbg_atten())
                    .hp_sleep_pd_cur() // pmu_ll_hp_set_current_power_off
                    .bit(self.hp_sys.bias.pd_cur())
                    .sleep() // pmu_ll_hp_set_bias_sleep_enable
                    .bit(self.hp_sys.bias.bias_sleep())
            });
            pmu().hp_sleep_hp_regulator0().modify(|_, w| {
                w.hp_sleep_hp_regulator_xpd() // pmu_ll_hp_set_regulator_xpd
                    .bit(self.hp_sys.regulator0.xpd())
                    .hp_sleep_hp_regulator_dbias() // pmu_ll_hp_set_regulator_dbias
                    .bits(self.hp_sys.regulator0.dbias())
            });
            pmu().hp_sleep_hp_regulator1().modify(|_, w| {
                w.hp_sleep_hp_regulator_drv_b() // pmu_ll_hp_set_regulator_driver_bar
                    .bits(self.hp_sys.regulator1.drv_b())
            });

            // LP SLEEP (lp_sleep_*)
            pmu().lp_sleep_bias().modify(|_, w| {
                w.lp_sleep_dbg_atten() // pmu_ll_lp_set_dbg_atten
                    .bits(self.lp_sys_sleep.bias.dbg_atten())
                    .lp_sleep_pd_cur() // pmu_ll_lp_set_current_power_off
                    .bit(self.lp_sys_sleep.bias.pd_cur())
                    .sleep() // pmu_ll_lp_set_bias_sleep_enable
                    .bit(self.lp_sys_sleep.bias.bias_sleep())
            });

            pmu().lp_sleep_lp_regulator0().modify(|_, w| {
                w.lp_sleep_lp_regulator_slp_xpd() // pmu_ll_lp_set_regulator_slp_xpd
                    .bit(self.lp_sys_sleep.regulator0.slp_xpd())
                    .lp_sleep_lp_regulator_xpd() // pmu_ll_lp_set_regulator_xpd
                    .bit(self.lp_sys_sleep.regulator0.xpd())
                    .lp_sleep_lp_regulator_slp_dbias() // pmu_ll_lp_set_regulator_sleep_dbias
                    .bits(self.lp_sys_sleep.regulator0.slp_dbias())
                    .lp_sleep_lp_regulator_dbias() // pmu_ll_lp_set_regulator_dbias
                    .bits(self.lp_sys_sleep.regulator0.dbias())
            });

            pmu().lp_sleep_lp_regulator1().modify(|_, w| {
                w.lp_sleep_lp_regulator_drv_b() // pmu_ll_lp_set_regulator_driver_bar
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
        unsafe {
            pmu().hp_sleep_hp_sys_cntl().modify(|_, w| {
                w.hp_sleep_dig_pad_slp_sel()
                    .bit(self.syscntl.dig_pad_slp_sel())
            })
        };
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

        unsafe {
            // HP SLEEP (hp_sleep_*)
            pmu()
                .hp_sleep_dig_power()
                .modify(|_, w| w.bits(self.hp_sys.dig_power.0));
            pmu()
                .hp_sleep_hp_ck_power()
                .modify(|_, w| w.bits(self.hp_sys.clk.0));
            pmu()
                .hp_sleep_xtal()
                .modify(|_, w| w.hp_sleep_xpd_xtal().bit(self.hp_sys.xtal.xpd_xtal()));

            // LP ACTIVE (hp_sleep_lp_*)
            pmu()
                .hp_sleep_lp_dig_power()
                .modify(|_, w| w.bits(self.lp_sys_active.dig_power.0));
            pmu()
                .hp_sleep_lp_ck_power()
                .modify(|_, w| w.bits(self.lp_sys_active.clk_power.0));

            // LP SLEEP (lp_sleep_*)
            pmu()
                .lp_sleep_lp_dig_power()
                .modify(|_, w| w.bits(self.lp_sys_sleep.dig_power.0));
            pmu()
                .lp_sleep_lp_ck_power()
                .modify(|_, w| w.bits(self.lp_sys_sleep.clk_power.0));
            pmu()
                .lp_sleep_xtal()
                .modify(|_, w| w.lp_sleep_xpd_xtal().bit(self.lp_sys_sleep.xtal.xpd_xtal()));
        }
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
    /// Number of cycles to wait for switching the ICG coйntrol.
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
            analog_wait_target_cycle: 2419,
            digital_power_supply_wait_cycle: 32,
            digital_power_up_wait_cycle: 32,
            modem_wakeup_wait_cycle: 20700,
            pll_stable_wait_cycle: 2,

            digital_power_down_wait_cycle: 0,
            modify_icg_cntl_wait_cycle: 0,
            switch_icg_cntl_wait_cycle: 0,
        },
        lp_sys: LpParam {
            min_slp_slow_clk_cycle: 10,
            analog_wait_target_cycle: 23,
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
            pmu().slp_wakeup_cntl3().modify(|_, w| {
                w.hp_min_slp_val() // pmu_ll_hp_set_min_sleep_cycle
                    .bits(self.hp_sys.min_slp_slow_clk_cycle)
                    .lp_min_slp_val() // pmu_ll_lp_set_min_sleep_cycle
                    .bits(self.lp_sys.min_slp_slow_clk_cycle)
            });

            pmu().slp_wakeup_cntl7().modify(|_, w| {
                w.ana_wait_target() // pmu_ll_hp_set_analog_wait_target_cycle
                    .bits(self.hp_sys.analog_wait_target_cycle)
            });

            pmu().power_wait_timer0().modify(|_, w| {
                w.dg_hp_wait_timer() // pmu_ll_hp_set_digital_power_supply_wait_cycle
                    .bits(self.hp_sys.digital_power_supply_wait_cycle)
                    .dg_hp_powerup_timer() // pmu_ll_hp_set_digital_power_up_wait_cycle
                    .bits(self.hp_sys.digital_power_up_wait_cycle)
            });

            pmu().power_wait_timer1().modify(|_, w| {
                w.dg_lp_wait_timer() // pmu_ll_lp_set_digital_power_supply_wait_cycle
                    .bits(self.lp_sys.digital_power_supply_wait_cycle)
                    .dg_lp_powerup_timer() // pmu_ll_lp_set_digital_power_up_wait_cycle
                    .bits(self.lp_sys.digital_power_up_wait_cycle)
            });

            pmu().slp_wakeup_cntl5().modify(|_, w| {
                w.lp_ana_wait_target() // pmu_ll_lp_set_analog_wait_target_cycle
                    .bits(self.lp_sys.analog_wait_target_cycle)
                    .modem_wait_target() // pmu_ll_hp_set_modem_wakeup_wait_cycle
                    .bits(self.hp_sys.modem_wakeup_wait_cycle)
            });
            pmu().power_ck_wait_cntl().modify(|_, w| {
                w.wait_xtl_stable() // pmu_ll_hp_set_xtal_stable_wait_cycle
                    .bits(self.hp_lp.xtal_stable_wait_cycle)
                    .wait_pll_stable() // pmu_ll_hp_set_pll_stable_wait_cycle
                    .bits(self.hp_sys.pll_stable_wait_cycle)
            });
        }
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

    fn rtc_clk_cal_fast(mut slowclk_cycles: u32) -> u32 {
        let xtal_freq = 40; // esp-idf has a very complicated way of determining this

        // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
        // So we need to divide the calibrate cycles of the FOSC for ECO1 and above
        // chips by 32 to avoid excessive calibration time.
        if Efuse::chip_revision() >= 1 {
            slowclk_cycles /= 32;
        }

        let xtal_cycles =
            RtcClock::calibrate_internal(RtcCalSel::RtcCalRcFast, slowclk_cycles) as u64;

        let divider: u64 = xtal_freq as u64 * slowclk_cycles as u64;
        let period_64: u64 = ((xtal_cycles << Self::RTC_CLK_CAL_FRACT) + divider / 2 - 1) / divider;
        (period_64 & (u32::MAX as u64)) as u32
    }

    fn new(_deep: bool) -> Self {
        // https://github.com/espressif/esp-idf/commit/e1d24ebd7f43c7c7ded183bc8800b20af3bf014b

        // Calibrate rtc slow clock
        // TODO: do an actual calibration instead of a read
        let slowclk_period = unsafe { lp_aon().store1().read().lp_aon_store1().bits() };

        // Calibrate rtc fast clock, only PMU supported chips sleep process is needed.
        const FAST_CLK_SRC_CAL_CYCLES: u32 = 2048;
        let fastclk_period = Self::rtc_clk_cal_fast(FAST_CLK_SRC_CAL_CYCLES);

        Self {
            sleep_time_adjustment: 0,
            slowclk_period,
            fastclk_period,
        }
    }

    fn light_sleep(pd_flags: PowerDownFlags) -> Self {
        const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 56;

        let mut this = Self::new(false);

        let sw = LIGHT_SLEEP_TIME_OVERHEAD_US; // TODO
        let hw = this.pmu_sleep_calculate_hw_wait_time(pd_flags);

        this.sleep_time_adjustment = sw + hw;

        this
    }

    fn deep_sleep() -> Self {
        let mut this = Self::new(true);

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

        let lp_hw_wait_time_us = MachineConstants::LP_MIN_SLP_TIME_US
            + MachineConstants::LP_ANALOG_WAIT_TIME_US
            + lp_clk_power_on_wait_time_us
            + lp_wakeup_wait_time_us
            + lp_clk_switch_time_us
            + MachineConstants::LP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::LP_POWER_UP_WAIT_TIME_US;

        // HP core hardware wait time, microsecond
        let hp_digital_power_up_wait_time_us = MachineConstants::HP_POWER_SUPPLY_WAIT_TIME_US
            + MachineConstants::HP_POWER_UP_WAIT_TIME_US;
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
            );

        #[rustfmt::skip] // ASCII art
        /* When the SOC wakeup (lp timer or GPIO wakeup) and Modem wakeup (Beacon wakeup) complete,
        * the soc wakeup will be delayed until the RF is turned on in Modem state.
        *
        *              modem wakeup                      TBTT, RF on by HW
        *                   |                                    |
        *                  \|/                                  \|/
        * PMU_HP_ACTIVE                                                                         /------
        * PMU_HP_MODEM                                           /------------//////////////////
        * PMU_HP_SLEEP   ----------------------//////////////////
        *                  /|\                /|\ /|\          /|\           /|\              /|\
        *                   |<- some hw wait ->|   |            |             |<- M2A switch ->|
        *                   |  slow cycles &   | soc wakeup     |                              |
        *                   |   FOSC cycles    |<- S2M switch ->|                              |
        *                   |                                                                  |
        *                   |<--      PMU guard time, also the maximum time for the SOC     -->|
        *                   |                           wake-up delay                          |
        */
        const CONFIG_ESP_WIFI_ENHANCED_LIGHT_SLEEP: bool = true;

        let (rf_on_protect_time_us, sync_time_us) = if CONFIG_ESP_WIFI_ENHANCED_LIGHT_SLEEP {
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

unsafe fn pmu<'a>() -> &'a esp32c6::pmu::RegisterBlock {
    &*esp32c6::PMU::ptr()
}

unsafe fn lp_aon<'a>() -> &'a esp32c6::lp_aon::RegisterBlock {
    &*esp32c6::LP_AON::ptr()
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

// Constants defined in `PMU_SLEEP_MC_DEFAULT()`
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
    // Unused, but defined in esp-idf. May be needed later.
    // const HP_REGDMA_A2S_WORK_TIME_US: u32 = 382;
    const HP_REGDMA_RF_ON_WORK_TIME_US: u32 = 70;
    // Unused, but defined in esp-idf. May be needed later.
    // const HP_REGDMA_RF_OFF_WORK_TIME_US: u32 = 23;
    const HP_XTAL_WAIT_STABLE_TIME_US: u32 = 250;
    const HP_PLL_WAIT_STABLE_TIME_US: u32 = 1;

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

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        Self::wake_io_reset();
    }

    fn wake_io_reset() {
        // loosely based on esp_deep_sleep_wakeup_io_reset

        let peripherals = unsafe {
            // We're stealing peripherals to do some uninitialization after waking up from
            // deep sleep. We have to be careful to only touch settings that were enabled
            // by deep sleep setup.
            Peripherals::steal()
        };
        let mut pins = peripherals.GPIO.pins();

        Ext1WakeupSource::wake_io_reset(&mut pins);
    }

    /// Finalize power-down flags, apply configuration based on the flags.
    pub(crate) fn apply(&mut self) {
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
            self.pd_flags.set_pd_xtal32k(true);
            self.pd_flags.set_pd_rc32k(true);
            self.pd_flags.set_pd_rc_fast(true);
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

        let wakeup_mask = wakeup_triggers.0 as u32;
        let reject_mask = if self.deep {
            0
        } else {
            // TODO: MODEM_REJECT if s_sleep_modem.wifi.phy_link != NULL
            let reject_mask = RTC_SLEEP_REJECT_MASK | MODEM_REJECT;
            wakeup_mask & reject_mask
        };

        let cpu_freq_config = SavedClockConfig::save();
        rtc_clk_cpu_freq_set_xtal();

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

        unsafe {
            // lp_aon_hal_inform_wakeup_type - tells ROM which wakeup stub to run
            lp_aon()
                .store9()
                .modify(|r, w| w.bits(r.bits() & !0x01 | self.deep as u32));

            // pmu_ll_hp_set_wakeup_enable
            pmu().slp_wakeup_cntl2().write(|w| w.bits(wakeup_mask));

            // pmu_ll_hp_set_reject_enable
            pmu().slp_wakeup_cntl1().modify(|_, w| {
                w.slp_reject_en()
                    .bit(true)
                    .sleep_reject_ena()
                    .bits(reject_mask)
            });

            // pmu_ll_hp_clear_reject_cause
            pmu()
                .slp_wakeup_cntl4()
                .write(|w| w.slp_reject_cause_clr().bit(true));

            pmu().int_clr().write(|w| {
                w.sw() // pmu_ll_hp_clear_sw_intr_status
                    .clear_bit_by_one()
                    .soc_sleep_reject() // pmu_ll_hp_clear_reject_intr_status
                    .clear_bit_by_one()
                    .soc_wakeup() // pmu_ll_hp_clear_wakeup_intr_status
                    .clear_bit_by_one()
            });

            // misc_modules_sleep_prepare

            // TODO: IDF-7370
            #[cfg(not(pmu))]
            if !(self.deep && wakeup_triggers.touch) {
                let saradc = &*esp32c6::APB_SARADC::ptr();
                saradc
                    .ctrl()
                    .modify(|_, w| w.saradc2_pwdet_drv().bit(false));
            }

            // Start entry into sleep mode

            // pmu_ll_hp_set_sleep_enable
            pmu().slp_wakeup_cntl0().write(|w| w.sleep_req().bit(true));

            // In pd_cpu lightsleep and deepsleep mode, we never get here
            loop {
                let int_raw = pmu().int_raw().read();
                if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                    break;
                }
            }
        }

        // esp-idf returns if the sleep was rejected, we don't return anything

        cpu_freq_config.restore();
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

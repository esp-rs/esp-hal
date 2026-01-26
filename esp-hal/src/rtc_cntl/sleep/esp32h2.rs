use core::ops::Not;

use crate::{
    clock::Clock,
    gpio::{AnyPin, Input, InputConfig, Pull, RtcPin},
    peripherals::APB_SARADC,
    rtc_cntl::{
        Rtc,
        RtcClock,
        rtc::{HpSysCntlReg, HpSysPower, LpSysPower},
        sleep::{Ext1WakeupSource, TimerWakeupSource, WakeSource, WakeTriggers, WakeupLevel},
    },
    soc::clocks::{ClockTree, CpuClkConfig, HpRootClkConfig, Timg0CalibrationClockConfig},
};

impl WakeSource for TimerWakeupSource {
    fn apply(
        &self,
        rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        _sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_timer(true);

        let lp_timer = unsafe { &*esp32h2::LP_TIMER::ptr() };
        let clock_freq = RtcClock::slow_freq();
        // TODO: maybe add sleep time adjustment like idf
        // TODO: maybe add check to prevent overflow?
        let clock_hz = clock_freq.frequency().as_hz() as u64;
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
                .write(|w| w.soc_wakeup_int_clr().set_bit());
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

    /// Resets the pins that had been configured as wakeup trigger to their default state.
    fn wake_io_reset() {
        fn uninit_pin(pin: impl RtcPin, wakeup_pins: u8) {
            if wakeup_pins & (1 << pin.rtc_number()) != 0 {
                pin.rtcio_pad_hold(false);
                pin.degrade().init_gpio();
            }
        }

        let wakeup_pins = Ext1WakeupSource::wakeup_pins();
        for_each_lp_function! {
            (($_rtc:ident, LP_GPIOn, $n:literal), $gpio:ident) => {
                uninit_pin(unsafe { $crate::peripherals::$gpio::steal() }, wakeup_pins);
            };
        }
    }
}

impl WakeSource for Ext1WakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        triggers.set_ext1(true);
        sleep_config.need_pd_top = true;

        // ext1_wakeup_prepare
        let mut pins = self.pins.borrow_mut();
        let mut pin_mask = 0u8;
        let mut level_mask = 0u8;
        for (pin, level) in pins.iter_mut() {
            pin_mask |= 1 << pin.rtc_number();
            level_mask |= match level {
                WakeupLevel::High => 1 << pin.rtc_number(),
                WakeupLevel::Low => 0,
            };

            pin.rtcio_pad_hold(true);
            Input::new(
                unsafe { AnyPin::steal(pin.number()) },
                InputConfig::default().with_pull(match level {
                    WakeupLevel::High => Pull::Down,
                    WakeupLevel::Low => Pull::Up,
                }),
            );
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
        // reset GPIOs to default state
        let mut pins = self.pins.borrow_mut();
        for (pin, _level) in pins.iter_mut() {
            pin.rtcio_pad_hold(false);
            unsafe { AnyPin::steal(pin.number()) }.init_gpio();
        }
    }
}

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
        unsafe {
            pmu().hp_sleep_hp_regulator0().modify(|_, w| {
                w.hp_sleep_hp_regulator_xpd() // pmu_ll_hp_set_regulator_xpd
                    .bit(self.regulator_xpd)
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
        unsafe {
            pmu().hp_sleep_sysclk().modify(|_, w| {
                w.hp_sleep_icg_sys_clock_en().bit(self.icg_func != 0) // pmu_ll_hp_set_icg_sysclk_enable
            });
            pmu().hp_sleep_icg_hp_func().modify(|_, w| {
                w.hp_sleep_dig_icg_func_en().bits(self.icg_func) // pmu_ll_hp_set_icg_func
            });
            if !self.deep_sleep {
                pmu().hp_sleep_hp_sys_cntl().modify(|_, w| {
                    w.hp_sleep_dig_pad_slp_sel()
                        .bit(self.syscntl.dig_pad_slp_sel()) // pmu_ll_hp_set_dig_pad_slp_sel
                });
            }
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

            pmu().slp_wakeup_cntl5().modify(|_, w| {
                w.lp_ana_wait_target() // pmu_ll_lp_set_analog_wait_target_cycle
                    .bits(self.lp_sys.analog_wait_target_cycle)
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

            pmu().power_ck_wait_cntl().modify(|_, w| {
                w.wait_xtl_stable() // pmu_ll_set_xtal_stable_wait_cycle
                    .bits(self.hp_lp.xtal_stable_wait_cycle)
                    .wait_pll_stable() // pmu_ll_set_pll_stable_wait_cycle
                    .bits(self.hp_sys.pll_stable_wait_cycle)
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

#[derive(Clone, Copy)]
struct SleepTimeConfig {
    sleep_time_adjustment: u32, // TODO: use this adjustment for calculating wakeup time
    slowclk_period: u32,
    fastclk_period: u32,
}

const CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ: u32 = 96;

impl SleepTimeConfig {
    const RTC_CLK_CAL_FRACT: u32 = 19;

    fn rtc_clk_cal_fast(slowclk_cycles: u32) -> u32 {
        RtcClock::calibrate(Timg0CalibrationClockConfig::RcFastDivClk, slowclk_cycles)
    }

    fn new() -> Self {
        let slowclk_period = unsafe { lp_aon().store1().read().data().bits() };

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
        const LIGHT_SLEEP_TIME_OVERHEAD_US: u32 = 9;

        let mut this = Self::new();

        let sw = LIGHT_SLEEP_TIME_OVERHEAD_US;
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

unsafe fn pmu<'a>() -> &'a esp32h2::pmu::RegisterBlock {
    unsafe { &*esp32h2::PMU::ptr() }
}

unsafe fn lp_aon<'a>() -> &'a esp32h2::lp_aon::RegisterBlock {
    unsafe { &*esp32h2::LP_AON::ptr() }
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

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        Self::wake_io_reset();
    }

    fn wake_io_reset() {
        Ext1WakeupSource::wake_io_reset();
    }

    /// Finalize power-down flags, apply configuration based on the flags.
    pub(crate) fn apply(&mut self) {
        if self.deep {
            // force-disable certain power domains
            self.pd_flags.set_pd_top(self.need_pd_top.not());
            self.pd_flags.set_pd_vddsdio(true);
            self.pd_flags.set_pd_modem(true);
            self.pd_flags.set_pd_cpu(true);
            self.pd_flags.set_pd_xtal(true);
            self.pd_flags.set_pd_rc_fast(true);
            self.pd_flags.set_pd_xtal32k(true);
        } else if self.need_pd_top {
            self.pd_flags.set_pd_top(false);
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

        let wakeup_mask = wakeup_triggers.0 as u32;
        let reject_mask = if self.deep {
            0
        } else {
            let reject_mask = RTC_SLEEP_REJECT_MASK;
            wakeup_mask & reject_mask
        };

        let cpu_freq_config = ClockTree::with(|clocks| {
            let cpu_freq_config = SavedClockConfig::save(clocks);
            crate::soc::clocks::configure_hp_root_clk(
                clocks,
                crate::soc::clocks::HpRootClkConfig::Xtal,
            );
            crate::soc::clocks::configure_cpu_clk(clocks, crate::soc::clocks::CpuClkConfig::new(0));
            cpu_freq_config
        });

        // misc_modules_sleep_prepare
        // TODO: IDF-7370

        APB_SARADC::regs()
            .ctrl()
            .modify(|_, w| w.saradc2_pwdet_drv().bit(false));

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

            pmu().int_clr().write(|w| {
                w.soc_wakeup() // pmu_ll_hp_clear_wakeup_intr_status
                    .clear_bit_by_one()
                    .soc_sleep_reject() // pmu_ll_hp_clear_reject_intr_status
                    .clear_bit_by_one()
            });

            // pmu_ll_hp_clear_reject_cause
            pmu()
                .slp_wakeup_cntl4()
                .write(|w| w.slp_reject_cause_clr().bit(true));

            // Start entry into sleep mode

            // pmu_ll_hp_set_sleep_enable
            pmu().slp_wakeup_cntl0().write(|w| w.sleep_req().bit(true));

            loop {
                let int_raw = pmu().int_raw().read();
                if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                    break;
                }
            }
        }

        // esp-idf returns if the sleep was rejected, we don't return anything

        ClockTree::with(|clocks| {
            cpu_freq_config.restore(clocks);
        });
    }

    /// Cleans up after sleep
    pub(crate) fn finish_sleep(&self) {
        Self::wake_io_reset();
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SavedClockConfig {
    /// The clock from which CPU clock is derived
    old_hp_root_clk: Option<HpRootClkConfig>,

    /// CPU divider
    old_cpu_divider: Option<CpuClkConfig>,
}

impl SavedClockConfig {
    pub(crate) fn save(clocks: &ClockTree) -> Self {
        let old_hp_root_clk = clocks.hp_root_clk();
        let old_cpu_divider = clocks.cpu_clk();

        SavedClockConfig {
            old_hp_root_clk,
            old_cpu_divider,
        }
    }

    // rtc_clk_cpu_freq_set_config
    pub(crate) fn restore(self, clocks: &mut ClockTree) {
        if let Some(old_hp_root_clk) = self.old_hp_root_clk {
            crate::soc::clocks::configure_hp_root_clk(clocks, old_hp_root_clk);
        }
        if let Some(old_cpu_divider) = self.old_cpu_divider {
            crate::soc::clocks::configure_cpu_clk(clocks, old_cpu_divider);
        }
    }
}

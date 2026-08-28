//! Light- and deep-sleep support for the ESP32-S31.

use core::ops::Not;

use crate::{
    peripherals::{LP_SYS as LP_AON, PMU},
    private::DropGuard,
    rtc_cntl::{
        Rtc,
        rtc::{HpAnalog, HpSysCntlReg, HpSysPower, LpAnalog, LpSysPower},
        sleep::{SleepKind, pmu_common::SleepTimeConfig},
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

        if !pd_flags.pd_xtal() {
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

    pub(crate) fn set_sleep_kind(&mut self, kind: SleepKind) {
        self.deep = kind == SleepKind::Deep;
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {}

    /// Finalize power-down flags, apply configuration based on the flags.
    pub(crate) fn apply(&mut self) {
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

    /// Configures the wakeup options and requests the sleep.
    ///
    /// The caller waits for the result of the request. The return value is a guard that restores
    /// what sleep entry changed for the sleep only, so the caller keeps it until the sleep ends.
    #[crate::ram]
    pub(crate) fn start_sleep(&self, wakeup_mask: u32, reject_mask: u32) -> impl Sized {
        let restore_clock_config = ClockTree::with(|clocks| {
            let old_cpu_root_clk = clocks.cpu_root_clk();

            clocks::configure_cpu_root_clk(clocks, CpuRootClkConfig::Xtal);

            DropGuard::new((), move |_| {
                ClockTree::with(|clocks| {
                    if let Some(old) = old_cpu_root_clk {
                        clocks::configure_cpu_root_clk(clocks, old);
                    }
                });
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

        PMU::regs().slp_wakeup_cntl0().write(|w| {
            w.slp_reject_en().bit(reject_mask != 0);
            w.sleep_req().bit(true)
        });

        restore_clock_config
    }

    /// Cleans up after sleep.
    ///
    /// Only a light sleep returns to the caller, and it needs no cleanup: the guard of
    /// [`Self::start_sleep`] restores the clock, and the wakeup sources keep their configuration.
    pub(crate) fn finish_sleep(&self) {}
}

use crate::{
    peripherals::{LP_AON, LP_AON_CLKRST, LP_TIMER, PMU},
    rtc_cntl::{
        Rtc,
        sleep::{TimerWakeupSource, WakeSource, WakeTriggers},
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

        let lp_timer = LP_TIMER::regs();
        let ticks = crate::clock::us_to_rtc_ticks(self.duration.as_micros() as u64);
        let now = rtc.time_since_boot_raw();
        let time_in_ticks = now + ticks;
        unsafe {
            // IDF order: clear int, write LOW, then write HIGH with tar_en
            // in the same write so the target latches atomically.
            lp_timer
                .int_clr()
                .write(|w| w.soc_wakeup().clear_bit_by_one());
            lp_timer.tar0_low().write(|w| {
                w.main_timer_tar_low0()
                    .bits((time_in_ticks & 0xffff_ffff) as u32)
            });
            lp_timer.tar0_high().write(|w| {
                w.main_timer_tar_high0()
                    .bits(((time_in_ticks >> 32) & 0xffff) as u16)
                    .main_timer_tar_en0()
                    .set_bit()
            });
            lp_timer.int_ena().modify(|_, w| w.soc_wakeup().set_bit());
        }
    }
}

// PMU_SLEEP_MC_DEFAULT + LSLP analog defaults from esp-idf pmu_param.c.
struct MachineConstants;

impl MachineConstants {
    const HP_MIN_SLP_TIME_US: u32 = 450;
    const HP_ANA_WAIT_PD_TOP_US: u32 = 260;
    const HP_POWER_SUPPLY_WAIT_US: u32 = 2;
    const HP_POWER_UP_WAIT_US: u32 = 26;
    const HP_PLL_STABLE_US: u32 = 50;
    const HP_XTAL_STABLE_US: u32 = 250;

    const LP_MIN_SLP_TIME_US: u32 = 450;
    const LP_ANALOG_WAIT_US: u32 = 154;
    const LP_POWER_SUPPLY_WAIT_US: u32 = 2;
    const LP_POWER_UP_WAIT_US: u32 = 2;

    const RC_FAST_HZ: u64 = 20_000_000;

    // PMU_SLEEP_PROTECT_HP_LP_SLEEP from esp_pmu.h
    const SLEEP_PROTECT_HP_LP_SLEEP: u8 = 2;

    // slp_wakeup_cntl2 wake bits, per IDF pmu_bit_defs.h.
    const LP_TIMER_WAKEUP_EN: u32 = 1 << 13;
    const UART0_WAKEUP_EN: u32 = 1 << 8;
    const UART1_WAKEUP_EN: u32 = 1 << 7;

    const HP_DCM_VSET_LSLP: u8 = 14;
    const HP_DBIAS_ACTIVE_DEFAULT: u8 = 24;
    const HP_REGULATOR_SLP_MEM_DBIAS_LSLP: u8 = 8;
    // Deviation from IDF's PMU_LP_DBIAS_LIGHTSLEEP_0V7 (= 12). On the
    // P4 EV Board v1.7, dbias=12 brown-outs ~530 ms into HP_SLEEP; 24
    // gives enough headroom.
    const LP_DBIAS_LIGHTSLEEP: u8 = 24;
    const HP_DBIAS_LIGHTSLEEP: u8 = 1;
}

/// Flags controlling which power domains drop to sleep state. Mirrors the
/// IDF `PMU_SLEEP_PD_*` flag set.
#[derive(Debug, Clone, Copy, Default)]
pub struct PowerDownFlags(u32);

impl PowerDownFlags {
    const PD_TOP: u32 = 1 << 0;
    const PD_VDDSDIO: u32 = 1 << 1;
    const PD_MODEM: u32 = 1 << 2;
    const PD_HP_PERIPH: u32 = 1 << 3;
    const PD_CPU: u32 = 1 << 4;
    const PD_HP_AON: u32 = 1 << 5;
    const PD_LP_PERIPH: u32 = 1 << 6;
    const PD_XTAL: u32 = 1 << 7;
    const PD_XTAL32K: u32 = 1 << 8;
    const PD_RC32K: u32 = 1 << 9;
    const PD_RC_FAST: u32 = 1 << 10;

    /// Returns `true` when no power-domain flag is set.
    pub fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Power down the HP TOP domain in sleep.
    pub fn pd_top(&self) -> bool {
        self.0 & Self::PD_TOP != 0
    }
    /// Power down the VDD_SPI rail in sleep.
    pub fn pd_vddsdio(&self) -> bool {
        self.0 & Self::PD_VDDSDIO != 0
    }
    /// Power down the modem (CNNT) domain in sleep.
    pub fn pd_modem(&self) -> bool {
        self.0 & Self::PD_MODEM != 0
    }
    /// Power down HP peripherals in sleep.
    pub fn pd_hp_periph(&self) -> bool {
        self.0 & Self::PD_HP_PERIPH != 0
    }
    /// Power down the HP CPU domain in sleep.
    pub fn pd_cpu(&self) -> bool {
        self.0 & Self::PD_CPU != 0
    }
    /// Power down the HP AON domain in sleep.
    pub fn pd_hp_aon(&self) -> bool {
        self.0 & Self::PD_HP_AON != 0
    }
    /// Power down LP peripherals in sleep.
    pub fn pd_lp_periph(&self) -> bool {
        self.0 & Self::PD_LP_PERIPH != 0
    }
    /// Power down XTAL in sleep.
    pub fn pd_xtal(&self) -> bool {
        self.0 & Self::PD_XTAL != 0
    }
    /// Power down XTAL32K in sleep.
    pub fn pd_xtal32k(&self) -> bool {
        self.0 & Self::PD_XTAL32K != 0
    }
    /// Power down RC32K in sleep.
    pub fn pd_rc32k(&self) -> bool {
        self.0 & Self::PD_RC32K != 0
    }
    /// Power down RC_FAST in sleep.
    pub fn pd_rc_fast(&self) -> bool {
        self.0 & Self::PD_RC_FAST != 0
    }

    /// Sets the HP TOP power-down flag.
    pub fn set_pd_top(&mut self, v: bool) {
        self.set(Self::PD_TOP, v);
    }
    /// Sets the VDD_SPI power-down flag.
    pub fn set_pd_vddsdio(&mut self, v: bool) {
        self.set(Self::PD_VDDSDIO, v);
    }
    /// Sets the modem (CNNT) power-down flag.
    pub fn set_pd_modem(&mut self, v: bool) {
        self.set(Self::PD_MODEM, v);
    }
    /// Sets the HP peripherals power-down flag.
    pub fn set_pd_hp_periph(&mut self, v: bool) {
        self.set(Self::PD_HP_PERIPH, v);
    }
    /// Sets the HP CPU power-down flag.
    pub fn set_pd_cpu(&mut self, v: bool) {
        self.set(Self::PD_CPU, v);
    }
    /// Sets the HP AON power-down flag.
    pub fn set_pd_hp_aon(&mut self, v: bool) {
        self.set(Self::PD_HP_AON, v);
    }
    /// Sets the LP peripherals power-down flag.
    pub fn set_pd_lp_periph(&mut self, v: bool) {
        self.set(Self::PD_LP_PERIPH, v);
    }
    /// Sets the XTAL power-down flag.
    pub fn set_pd_xtal(&mut self, v: bool) {
        self.set(Self::PD_XTAL, v);
    }
    /// Sets the XTAL32K power-down flag.
    pub fn set_pd_xtal32k(&mut self, v: bool) {
        self.set(Self::PD_XTAL32K, v);
    }
    /// Sets the RC32K power-down flag.
    pub fn set_pd_rc32k(&mut self, v: bool) {
        self.set(Self::PD_RC32K, v);
    }
    /// Sets the RC_FAST power-down flag.
    pub fn set_pd_rc_fast(&mut self, v: bool) {
        self.set(Self::PD_RC_FAST, v);
    }

    fn set(&mut self, mask: u32, v: bool) {
        if v {
            self.0 |= mask;
        } else {
            self.0 &= !mask;
        }
    }
}

// HP_SLEEP / LP_ACTIVE / LP_SLEEP power/clk/xtal -- pmu_sleep_power_init.

/// HP_SLEEP / LP_ACTIVE / LP_SLEEP power-domain programming.
#[derive(Clone, Copy)]
pub struct PowerSleepConfig {
    /// Per-sleep power-down flags.
    pub pd_flags: PowerDownFlags,
    /// `true` for deep sleep (only affects `dcdc_switch_pd_en`).
    pub deep: bool,
}

impl PowerSleepConfig {
    /// Builds the light-sleep config from `pd_flags`.
    pub fn defaults(pd_flags: PowerDownFlags) -> Self {
        Self {
            pd_flags,
            deep: false,
        }
    }

    /// Builds the deep-sleep config.
    pub fn defaults_deep_sleep(pd_flags: PowerDownFlags) -> Self {
        Self {
            pd_flags,
            deep: true,
        }
    }

    /// Applies the HP_SLEEP / LP_ACTIVE / LP_SLEEP power-domain registers.
    pub fn apply(&self) {
        let pmu = PMU::regs();
        let pd = self.pd_flags;
        unsafe {
            // dcdc_switch_pd_en: clear for light sleep (DCDC stays on
            // across sleep), set for deep.
            pmu.hp_sleep_dig_power().write(|w| {
                w.hp_sleep_hp_mem_dslp()
                    .clear_bit()
                    .hp_sleep_pd_hp_mem_pd_en()
                    .clear_bit()
                    .hp_sleep_pd_cnnt_pd_en()
                    .bit(pd.pd_modem())
                    .hp_sleep_pd_top_pd_en()
                    .bit(pd.pd_top())
                    .hp_sleep_dcdc_switch_pd_en()
                    .bit(self.deep)
            });

            pmu.hp_sleep_hp_ck_power().write(|w| {
                w.hp_sleep_i2c_iso_en()
                    .set_bit()
                    .hp_sleep_i2c_retention()
                    .set_bit()
                    .hp_sleep_xpd_pll_i2c()
                    .bits(1)
                    .hp_sleep_xpd_pll()
                    .bits(0)
            });

            pmu.hp_sleep_xtal()
                .write(|w| w.hp_sleep_xpd_xtal().bit(!pd.pd_xtal()));

            pmu.hp_sleep_lp_dig_power().write(|w| {
                w.hp_sleep_lp_pad_slp_sel()
                    .clear_bit()
                    .hp_sleep_bod_source_sel()
                    .clear_bit()
                    .hp_sleep_vddbat_mode()
                    .bits(0)
                    .hp_sleep_lp_mem_dslp()
                    .clear_bit()
                    .hp_sleep_pd_lp_peri_pd_en()
                    .bit(pd.pd_lp_periph())
            });
            pmu.hp_sleep_lp_ck_power().write(|w| {
                w.hp_sleep_xpd_lppll()
                    .clear_bit()
                    .hp_sleep_xpd_xtal32k()
                    .clear_bit()
                    .hp_sleep_xpd_rc32k()
                    .clear_bit()
                    .hp_sleep_xpd_fosc_clk()
                    .set_bit()
                    .hp_sleep_pd_osc_clk()
                    .clear_bit()
            });

            pmu.lp_sleep_lp_dig_power().write(|w| {
                w.lp_sleep_lp_pad_slp_sel()
                    .clear_bit()
                    .lp_sleep_bod_source_sel()
                    .clear_bit()
                    .lp_sleep_vddbat_mode()
                    .bits(0)
                    .lp_sleep_lp_mem_dslp()
                    .clear_bit()
                    .lp_sleep_pd_lp_peri_pd_en()
                    .bit(pd.pd_lp_periph())
            });
            pmu.lp_sleep_lp_ck_power().write(|w| {
                w.lp_sleep_xpd_lppll()
                    .clear_bit()
                    .lp_sleep_xpd_xtal32k()
                    .bit(!pd.pd_xtal32k())
                    .lp_sleep_xpd_rc32k()
                    .bit(!pd.pd_rc32k())
                    .lp_sleep_xpd_fosc_clk()
                    .bit(!pd.pd_rc_fast())
                    .lp_sleep_pd_osc_clk()
                    .clear_bit()
            });
            pmu.lp_sleep_xtal()
                .write(|w| w.lp_sleep_xpd_xtal().bit(!pd.pd_xtal()));
        }
    }
}

// HP_SLEEP + LP_SLEEP bias/regulator -- pmu_sleep_analog_init.

/// HP_SLEEP / LP_SLEEP analog (bias + regulator) programming.
#[derive(Clone, Copy)]
pub struct AnalogSleepConfig {
    /// Per-sleep power-down flags.
    pub pd_flags: PowerDownFlags,
    /// `true` for deep sleep.
    pub deep: bool,
}

impl AnalogSleepConfig {
    /// Light-sleep analog defaults.
    pub fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        Self {
            pd_flags,
            deep: false,
        }
    }

    /// Deep-sleep analog defaults.
    pub fn defaults_deep_sleep() -> Self {
        Self {
            pd_flags: PowerDownFlags::default(),
            deep: true,
        }
    }

    /// Writes the HP_SLEEP and LP_SLEEP `bias` / `regulator0/1` registers.
    pub fn apply(&self) {
        let pmu = PMU::regs();

        let xtal_drops = self.deep || self.pd_flags.pd_xtal();

        let hp_dcm_vset = if self.deep {
            0
        } else {
            MachineConstants::HP_DCM_VSET_LSLP
        };
        let hp_pd_cur = !self.deep && xtal_drops;

        let hp_dbias = if self.deep {
            0
        } else if self.pd_flags.pd_xtal() {
            MachineConstants::HP_DBIAS_LIGHTSLEEP
        } else {
            MachineConstants::HP_DBIAS_ACTIVE_DEFAULT
        };
        let hp_slp_mem_dbias = if self.deep {
            0
        } else {
            MachineConstants::HP_REGULATOR_SLP_MEM_DBIAS_LSLP
        };

        let lp_pd_cur = xtal_drops;
        let lp_dbg_atten: u8 = if self.deep { 12 } else { 0 };

        let lp_dbias = if self.deep {
            23
        } else {
            MachineConstants::LP_DBIAS_LIGHTSLEEP
        };

        unsafe {
            pmu.hp_sleep_bias().write(|w| {
                w.hp_sleep_dcm_vset()
                    .bits(hp_dcm_vset)
                    .hp_sleep_dcm_mode()
                    .bits(1)
                    .hp_sleep_xpd_bias()
                    .clear_bit()
                    .hp_sleep_dbg_atten()
                    .bits(0)
                    .hp_sleep_pd_cur()
                    .bit(hp_pd_cur)
                    .sleep()
                    .bit(hp_pd_cur)
            });
            pmu.hp_sleep_hp_regulator0().write(|w| {
                w.hp_sleep_hp_regulator_slp_mem_xpd()
                    .clear_bit()
                    .hp_sleep_hp_regulator_slp_logic_xpd()
                    .clear_bit()
                    .hp_sleep_hp_regulator_xpd()
                    .clear_bit()
                    .hp_sleep_hp_regulator_slp_mem_dbias()
                    .bits(hp_slp_mem_dbias)
                    .hp_sleep_hp_regulator_slp_logic_dbias()
                    .bits(0)
                    .hp_sleep_hp_regulator_dbias()
                    .bits(hp_dbias)
            });
            pmu.hp_sleep_hp_regulator1()
                .write(|w| w.hp_sleep_hp_regulator_drv_b().bits(0));
            pmu.lp_sleep_bias().write(|w| {
                w.lp_sleep_xpd_bias()
                    .clear_bit()
                    .lp_sleep_dbg_atten()
                    .bits(lp_dbg_atten)
                    .lp_sleep_pd_cur()
                    .bit(lp_pd_cur)
                    .sleep()
                    .bit(lp_pd_cur)
            });
            pmu.lp_sleep_lp_regulator0().write(|w| {
                w.lp_sleep_lp_regulator_slp_xpd()
                    .clear_bit()
                    .lp_sleep_lp_regulator_xpd()
                    .set_bit()
                    .lp_sleep_lp_regulator_slp_dbias()
                    .bits(0)
                    .lp_sleep_lp_regulator_dbias()
                    .bits(lp_dbias)
            });
            pmu.lp_sleep_lp_regulator1()
                .write(|w| w.lp_sleep_lp_regulator_drv_b().bits(0));
        }
    }
}

// HP_SLEEP hp_sys_cntl -- pmu_sleep_digital_init.

/// HP_SLEEP digital (`hp_sys_cntl`) programming.
#[derive(Clone, Copy)]
pub struct DigitalSleepConfig {
    /// Per-sleep power-down flags.
    pub pd_flags: PowerDownFlags,
}

impl DigitalSleepConfig {
    /// Light-sleep digital defaults.
    pub fn defaults_light_sleep(pd_flags: PowerDownFlags) -> Self {
        Self { pd_flags }
    }

    /// Writes the HP_SLEEP `hp_sys_cntl` register.
    pub fn apply(&self) {
        let pmu = PMU::regs();
        // dig_pad_slp_sel: set when TOP stays powered, cleared when it drops.
        let dig_pad_slp = !self.pd_flags.pd_top();
        pmu.hp_sleep_hp_sys_cntl().write(|w| {
            w.hp_sleep_uart_wakeup_en()
                .set_bit()
                .hp_sleep_lp_pad_hold_all()
                .clear_bit()
                .hp_sleep_hp_pad_hold_all()
                .clear_bit()
                .hp_sleep_dig_pad_slp_sel()
                .bit(dig_pad_slp)
                .hp_sleep_dig_pause_wdt()
                .set_bit()
                .hp_sleep_dig_cpu_stall()
                .set_bit()
        });
    }
}

// Wait counters -- slp_wakeup_cntl3/5/7, power_wait_timer0/1,
// power_ck_wait_cntl. Cycles derived from PMU_SLEEP_MC_DEFAULT us values.

/// Sleep wait-counter programming.
#[derive(Clone, Copy)]
pub struct ParamSleepConfig;

impl ParamSleepConfig {
    /// Default wait-counter config.
    pub fn defaults(_pd_flags: PowerDownFlags) -> Self {
        Self
    }

    /// Writes the `slp_wakeup_cntl3/5/7`, `power_wait_timer0/1`, and
    /// `power_ck_wait_cntl` registers.
    pub fn apply(&self) {
        let pmu = PMU::regs();

        let hp_min_slp = us_to_slowclk(MachineConstants::HP_MIN_SLP_TIME_US as u64) as u8;
        let lp_min_slp = us_to_slowclk(MachineConstants::LP_MIN_SLP_TIME_US as u64) as u8;
        let hp_ana_wait = us_to_slowclk(MachineConstants::HP_ANA_WAIT_PD_TOP_US as u64) as u32;
        let lp_ana_wait = us_to_slowclk(MachineConstants::LP_ANALOG_WAIT_US as u64) as u8;

        let hp_dig_supply = us_to_fastclk(MachineConstants::HP_POWER_SUPPLY_WAIT_US as u64) as u32;
        let hp_dig_powerup = us_to_fastclk(MachineConstants::HP_POWER_UP_WAIT_US as u64) as u32;
        let lp_dig_supply = us_to_fastclk(MachineConstants::LP_POWER_SUPPLY_WAIT_US as u64) as u32;
        let lp_dig_powerup = us_to_fastclk(MachineConstants::LP_POWER_UP_WAIT_US as u64) as u32;

        let xtal_stable = us_to_slowclk(MachineConstants::HP_XTAL_STABLE_US as u64) as u32;
        let pll_stable = us_to_fastclk(MachineConstants::HP_PLL_STABLE_US as u64) as u32;

        unsafe {
            pmu.slp_wakeup_cntl3().modify(|_, w| {
                w.sleep_prt_sel()
                    .bits(MachineConstants::SLEEP_PROTECT_HP_LP_SLEEP)
                    .hp_min_slp_val()
                    .bits(hp_min_slp.max(2))
                    .lp_min_slp_val()
                    .bits(lp_min_slp.max(2))
            });
            pmu.slp_wakeup_cntl5()
                .modify(|_, w| w.lp_ana_wait_target().bits(lp_ana_wait));
            pmu.slp_wakeup_cntl7()
                .modify(|_, w| w.ana_wait_target().bits(hp_ana_wait as u16));

            pmu.power_wait_timer0().write(|w| {
                w.dg_hp_powerdown_timer()
                    .bits(0)
                    .dg_hp_powerup_timer()
                    .bits(hp_dig_powerup as u16)
                    .dg_hp_wait_timer()
                    .bits(hp_dig_supply as u16)
            });
            pmu.power_wait_timer1().write(|w| {
                w.dg_lp_powerdown_timer()
                    .bits(0)
                    .dg_lp_powerup_timer()
                    .bits(lp_dig_powerup as u16)
                    .dg_lp_wait_timer()
                    .bits(lp_dig_supply as u16)
            });
            pmu.power_ck_wait_cntl().write(|w| {
                w.pmu_wait_xtl_stable()
                    .bits(xtal_stable as u16)
                    .pmu_wait_pll_stable()
                    .bits(pll_stable as u16)
            });
        }
    }
}

fn us_to_slowclk(us: u64) -> u64 {
    crate::clock::us_to_rtc_ticks(us)
}

fn us_to_fastclk(us: u64) -> u64 {
    us.saturating_mul(MachineConstants::RC_FAST_HZ) / 1_000_000
}

// REGI2C analog regulator init, mirrors esp-idf rtc_clk_init.
fn rtc_clk_regi2c_init() {
    use crate::soc::regi2c;

    regi2c::I2C_DIG_REG_FORCE_RTC_DREG.write_field(1);
    regi2c::I2C_DIG_REG_FORCE_DIG_DREG.write_field(1);
    regi2c::I2C_DIG_REG_XPD_RTC_REG.write_field(0);
    regi2c::I2C_DIG_REG_XPD_DIG_REG.write_field(0);

    regi2c::I2C_BIAS_OR_FORCE_XPD_CK.write_field(0);
    regi2c::I2C_BIAS_OR_FORCE_XPD_REF_OUT_BUF.write_field(0);
    regi2c::I2C_BIAS_OR_FORCE_XPD_IPH.write_field(0);
    regi2c::I2C_BIAS_OR_FORCE_XPD_VGATE_BUF.write_field(0);
}

// Mirrors pmu_power_domain_force_default: clear all force_* overrides.
fn pmu_power_domain_force_default() {
    let pmu = PMU::regs();
    unsafe {
        pmu.power_pd_top_cntl().write(|w| {
            w.force_top_reset()
                .clear_bit()
                .force_top_iso()
                .clear_bit()
                .force_top_pu()
                .clear_bit()
                .force_top_no_reset()
                .clear_bit()
                .force_top_no_iso()
                .clear_bit()
                .force_top_pd()
                .clear_bit()
        });
        pmu.power_pd_cnnt_cntl().write(|w| {
            w.force_cnnt_reset()
                .clear_bit()
                .force_cnnt_iso()
                .clear_bit()
                .force_cnnt_pu()
                .clear_bit()
                .force_cnnt_no_reset()
                .clear_bit()
                .force_cnnt_no_iso()
                .clear_bit()
                .force_cnnt_pd()
                .clear_bit()
        });
        pmu.power_pd_hpmem_cntl().write(|w| {
            w.force_hp_mem_reset()
                .clear_bit()
                .force_hp_mem_iso()
                .clear_bit()
                .force_hp_mem_pu()
                .clear_bit()
                .force_hp_mem_no_reset()
                .clear_bit()
                .force_hp_mem_no_iso()
                .clear_bit()
                .force_hp_mem_pd()
                .clear_bit()
        });
        // CPU domain (v3.x-only, exposed by PAC `551af1c3c`+).
        pmu.power_pd_hp_cpu_cntl().write(|w| {
            w.force_hp_cpu_reset()
                .clear_bit()
                .force_hp_cpu_iso()
                .clear_bit()
                .force_hp_cpu_pu()
                .clear_bit()
                .force_hp_cpu_no_reset()
                .clear_bit()
                .force_hp_cpu_no_iso()
                .clear_bit()
                .force_hp_cpu_pd()
                .clear_bit()
        });

        // Isolate all memory banks to limit leakage when HPMEM is off.
        pmu.power_pd_hpmem_mask()
            .write(|w| w.xpd_hp_mem_mask().bits(0).pd_hp_mem_mask().bits(0));

        pmu.power_pd_lpperi_cntl().write(|w| {
            w.force_lp_peri_reset()
                .clear_bit()
                .force_lp_peri_iso()
                .clear_bit()
                .force_lp_peri_pu()
                .clear_bit()
                .force_lp_peri_no_reset()
                .clear_bit()
                .force_lp_peri_no_iso()
                .clear_bit()
                .force_lp_peri_pd()
                .clear_bit()
        });

        pmu.power_dcdc_switch().write(|w| {
            w.force_dcdc_switch_pu()
                .clear_bit()
                .force_dcdc_switch_pd()
                .clear_bit()
        });
    }
}

// CACHE_MAP_L1_DCACHE = BIT(4) per esp32p4 rom/cache.h.
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

// Mirrors pmu_sleep_cache_sync_items. ROM invalidate APIs can't be used
// because the call dirties the stack on its way back.
fn pmu_sleep_writeback_l1_dcache() {
    let cache = unsafe { &*crate::pac::CACHE::PTR };
    cache.sync_addr().write(|w| unsafe { w.bits(0) });
    cache.sync_size().write(|w| unsafe { w.bits(0) });
    cache
        .sync_map()
        .write(|w| unsafe { w.sync_map().bits(CACHE_MAP_L1_DCACHE as u8) });
    cache
        .sync_ctrl()
        .modify(|_, w| unsafe { w.sync_rgid().bits(0) }.writeback_ena().set_bit());
    while !cache.sync_ctrl().read().sync_done().bit_is_set() {
        core::hint::spin_loop();
    }
}

// CPLL is gated in HP_SLEEP, so the FSM needs CPU on XTAL first.
const HP_ROOT_CLK_SRC_XTAL: u8 = 0;

fn save_and_switch_cpu_to_xtal() -> u8 {
    let clkrst = LP_AON_CLKRST::regs();
    let saved = clkrst
        .lp_aonclkrst_hp_clk_ctrl()
        .read()
        .lp_aonclkrst_hp_root_clk_src_sel()
        .bits();
    unsafe {
        clkrst.lp_aonclkrst_hp_clk_ctrl().modify(|_, w| {
            w.lp_aonclkrst_hp_root_clk_src_sel()
                .bits(HP_ROOT_CLK_SRC_XTAL)
        });
    }
    saved
}

fn restore_cpu_root_clk(saved: u8) {
    unsafe {
        LP_AON_CLKRST::regs()
            .lp_aonclkrst_hp_clk_ctrl()
            .modify(|_, w| w.lp_aonclkrst_hp_root_clk_src_sel().bits(saved));
    }
}

// MPLL must be off in HP_SLEEP. TODO: also re-run rtc_clk_mpll_configure
// on wake if PSRAM glitches.
fn rtc_clk_mpll_disable() {
    PMU::regs()
        .rf_pwc()
        .modify(|_, w| w.mspi_phy_xpd().clear_bit());
}

fn rtc_clk_mpll_enable() {
    PMU::regs()
        .rf_pwc()
        .modify(|_, w| w.mspi_phy_xpd().set_bit());
}

/// Configuration block for ESP32-P4 RTC sleep.
#[derive(Debug, Clone, Copy, Default)]
pub struct RtcSleepConfig {
    deep: bool,
    pd_flags: PowerDownFlags,
}

impl RtcSleepConfig {
    /// Returns `true` when deep sleep is requested.
    pub fn deep_slp(&self) -> bool {
        self.deep
    }

    /// Builds a deep-sleep configuration with all power domains marked
    /// for power-down.
    pub fn deep() -> Self {
        let mut pd_flags = PowerDownFlags::default();
        pd_flags.set_pd_top(true);
        pd_flags.set_pd_vddsdio(true);
        pd_flags.set_pd_modem(true);
        pd_flags.set_pd_hp_periph(true);
        pd_flags.set_pd_cpu(true);
        pd_flags.set_pd_hp_aon(true);
        pd_flags.set_pd_lp_periph(true);
        pd_flags.set_pd_xtal(true);
        pd_flags.set_pd_xtal32k(true);
        pd_flags.set_pd_rc32k(true);
        pd_flags.set_pd_rc_fast(true);
        Self {
            deep: true,
            pd_flags,
        }
    }

    /// Per-sleep power-down flags.
    pub fn pd_flags(&self) -> PowerDownFlags {
        self.pd_flags
    }

    /// Override per-sleep power-down flags.
    pub fn set_pd_flags(&mut self, flags: PowerDownFlags) {
        self.pd_flags = flags;
    }

    pub(crate) fn base_settings(_rtc: &Rtc<'_>) {
        // pmu_init -- flag-independent stamping.
        rtc_clk_regi2c_init();
        pmu_power_domain_force_default();

        let pmu = PMU::regs();
        unsafe {
            pmu.slp_wakeup_cntl3().modify(|_, w| {
                w.sleep_prt_sel()
                    .bits(MachineConstants::SLEEP_PROTECT_HP_LP_SLEEP)
            });
        }
    }

    pub(crate) fn apply(&mut self) {
        // Built inside start_sleep so callers can still mutate pd_flags.
    }

    pub(crate) fn start_sleep(&self, wakeup_triggers: WakeTriggers) {
        let saved_root_clk = save_and_switch_cpu_to_xtal();

        if self.deep {
            PowerSleepConfig::defaults_deep_sleep(self.pd_flags).apply();
            AnalogSleepConfig::defaults_deep_sleep().apply();
        } else {
            PowerSleepConfig::defaults(self.pd_flags).apply();
            AnalogSleepConfig::defaults_light_sleep(self.pd_flags).apply();
            DigitalSleepConfig::defaults_light_sleep(self.pd_flags).apply();
        }
        ParamSleepConfig::defaults(self.pd_flags).apply();

        let mut wakeup_mask = 0u32;
        if wakeup_triggers.timer() {
            wakeup_mask |= MachineConstants::LP_TIMER_WAKEUP_EN;
        }
        if wakeup_triggers.uart0() {
            wakeup_mask |= MachineConstants::UART0_WAKEUP_EN;
        }
        if wakeup_triggers.uart1() {
            wakeup_mask |= MachineConstants::UART1_WAKEUP_EN;
        }
        // Light sleep: any wake source also acts as reject (deep can't reject).
        let reject_mask = if self.deep { 0 } else { wakeup_mask };

        // LP_AON.lp_store8[0] = RTC_SLEEP_MODE_REG -- tells the ROM stub.
        let lp_aon = LP_AON::regs();
        unsafe {
            lp_aon.lp_store8().modify(|r, w| {
                let base = r.bits() & !0x1;
                w.bits(base | (self.deep as u32))
            });

            let pmu = PMU::regs();
            pmu.slp_wakeup_cntl2().write(|w| w.bits(wakeup_mask));

            pmu.slp_wakeup_cntl1().modify(|_, w| {
                w.sleep_reject_ena()
                    .bits(reject_mask)
                    .slp_reject_en()
                    .set_bit()
            });

            pmu.int_clr().write(|w| {
                w.sw()
                    .clear_bit_by_one()
                    .soc_sleep_reject()
                    .clear_bit_by_one()
                    .soc_wakeup()
                    .clear_bit_by_one()
            });
            pmu.slp_wakeup_cntl4()
                .write(|w| w.slp_reject_cause_clr().set_bit());
        }

        // Writeback before sleep_req (twice for !deep, since MPLL disable
        // can dirty further lines).
        pmu_sleep_writeback_l1_dcache();
        if !self.deep {
            rtc_clk_mpll_disable();
            pmu_sleep_writeback_l1_dcache();
        }

        let pmu = PMU::regs();
        pmu.imm_pad_hold_all()
            .write(|w| w.tie_high_pad_slp_sel().set_bit());

        pmu.slp_wakeup_cntl0().write(|w| w.sleep_req().bit(true));

        // Bounded so a broken wake doesn't hang the test bin.
        let mut spin = 0u64;
        const SPIN_LIMIT: u64 = 2_000_000_000;
        loop {
            let int_raw = pmu.int_raw().read();
            if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                break;
            }
            spin += 1;
            if spin >= SPIN_LIMIT {
                break;
            }
        }

        pmu.imm_pad_hold_all()
            .write(|w| w.tie_low_pad_slp_sel().set_bit());

        if !self.deep {
            rtc_clk_mpll_enable();
        }

        restore_cpu_root_clk(saved_root_clk);
    }

    pub(crate) fn finish_sleep(&self) {
        PMU::regs().int_clr().write(|w| {
            w.sw()
                .clear_bit_by_one()
                .soc_sleep_reject()
                .clear_bit_by_one()
                .soc_wakeup()
                .clear_bit_by_one()
        });
    }
}

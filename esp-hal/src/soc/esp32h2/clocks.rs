//! Clock tree definitions and implementations for ESP32-H2.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (130k RC_SLOW and 8M RC_FAST) are not calibrated here, this system can
//!   only give a rough estimate of their frequency. They can be calibrated separately using a known
//!   crystal frequency.
//! - Some of the SOC capabilities are not implemented.
#![allow(dead_code, reason = "Some of this is bound to be unused")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use crate::{
    peripherals::{I2C_ANA_MST, LP_CLKRST, MODEM_LPCON, PCR, PMU, TIMG0, TIMG1},
    soc::regi2c,
};

define_clock_tree_types!();

/// Clock configuration options.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum CpuClock {
    /// 96 MHz CPU clock
    #[default]
    _96MHz = 96,
}

impl CpuClock {
    const PRESET_96: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::Pll96),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(0)),
        apb_clk: Some(ApbClkConfig::new(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_96MHz => CpuClock::PRESET_96,
        }
    }
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self::from(CpuClock::default())
    }
}

impl ClockConfig {
    pub(crate) fn try_get_preset(self) -> Option<CpuClock> {
        match self {
            v if v == CpuClock::PRESET_96 => Some(CpuClock::_96MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(mut self) {
        if self.xtal_clk.is_none() {
            self.xtal_clk = Some(XtalClkConfig::_32);
        }

        self.apply();
    }
}

fn clk_ll_bus_update() {
    PCR::regs()
        .bus_clk_update()
        .modify(|_, w| w.bus_clock_update().bit(true));

    // reg_get_bit
    while PCR::regs()
        .bus_clk_update()
        .read()
        .bus_clock_update()
        .bit_is_set()
    {}
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, _config: XtalClkConfig) {
    // The stored configuration affects PLL settings instead.
}

// PLL_F96M_CLK

fn enable_pll_f96m_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
            w.tie_high_xpd_bb_i2c().set_bit();
            w.tie_high_xpd_bbpll().set_bit();
            w.tie_high_xpd_bbpll_i2c().set_bit()
        });
        PMU::regs()
            .imm_hp_ck_power()
            .modify(|_, w| w.tie_high_global_bbpll_icg().set_bit());
    } else {
        PMU::regs()
            .imm_hp_ck_power()
            .modify(|_, w| w.tie_low_global_bbpll_icg().set_bit());
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
            w.tie_low_xpd_bbpll().set_bit();
            w.tie_low_xpd_bbpll_i2c().set_bit()
        });

        return;
    }

    // Enable I2C master clock
    MODEM_LPCON::regs()
        .clk_conf_force_on()
        .modify(|_, w| w.clk_i2c_mst_fo().set_bit());

    // Set I2C clock to 96MHz
    const MODEM_LPCON_CLK_I2C_SEL_96M: u32 = 1 << 0; // FIXME add to PAC
    MODEM_LPCON::regs()
        .clk_conf()
        .modify(|r, w| unsafe { w.bits(r.bits() | MODEM_LPCON_CLK_I2C_SEL_96M) });

    // BPPLL calibration start
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    regi2c::I2C_BBPLL_OC_REF_DIV.write_field(0);
    regi2c::I2C_BBPLL_OC_DIV.write_field(1);
    regi2c::I2C_BBPLL_OC_DHREF_SEL.write_field(3);
    regi2c::I2C_BBPLL_OC_DLREF_SEL.write_field(1);

    // WAIT CALIBRATION DONE
    while I2C_ANA_MST::regs()
        .ana_conf0()
        .read()
        .cal_done()
        .bit_is_clear()
    {}

    // workaround bbpll calibration might stop early
    crate::rom::ets_delay_us(10);

    // BBPLL CALIBRATION STOP
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().set_bit();
        w.bbpll_stop_force_low().clear_bit()
    });
}

// PLL_F64M_CLK - called Flash PLL in esp-idf

fn enable_pll_f64m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_F48M_CLK

fn enable_pll_f48m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_fosc_clk().bit(en));
    // Enable for digital part
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_fosc().bit(en));
}

// XTAL32K_CLK

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_xtal32k().bit(en));

    // Enable for digital part
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_xtal32k().bit(en));
}

// OSC_SLOW_CLK

fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    unimplemented!()
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        // SCK_DCAP value controls tuning of 136k clock. The higher the value of DCAP, the lower the
        // frequency. There is no separate enable bit, just make sure the calibration value is set.
        const RTC_CNTL_SCK_DCAP_DEFAULT: u8 = 85;
        crate::soc::regi2c::I2C_PMU_OC_SCK_DCAP.write_reg(RTC_CNTL_SCK_DCAP_DEFAULT);
    }

    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_rc32k().bit(en));

    // Enable for digital part
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_osc32k().bit(en));
}

// PLL_LP_CLK

fn enable_pll_lp_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_lppll().bit(en));
}

// HP_ROOT_CLK

fn enable_hp_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do
}

fn configure_hp_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<HpRootClkConfig>,
    new_selector: HpRootClkConfig,
) {
    PCR::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_selector {
            HpRootClkConfig::Pll96 => 1,
            HpRootClkConfig::Pll64 => 3,
            HpRootClkConfig::Xtal => 0,
            HpRootClkConfig::RcFast => 2,
        })
    });

    // "When selecting the clock source of HP_ROOT_CLK, or configuring the clock divisor for CPU_CLK
    // and AHB_CLK ..."
    clk_ll_bus_update();
}

// CPU_CLK

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do
}

fn configure_cpu_clk_impl(_clocks: &mut ClockTree, new_config: CpuClkConfig) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits(new_config.value() as u8) });

    // "When selecting the clock source of HP_ROOT_CLK, or configuring the clock divisor for CPU_CLK
    // and AHB_CLK ..."
    clk_ll_bus_update();
}

// AHB_CLK

fn enable_ahb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do
}

fn configure_ahb_clk_impl(_clocks: &mut ClockTree, new_config: AhbClkConfig) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_div_num().bits(new_config.value() as u8) });

    // "When selecting the clock source of HP_ROOT_CLK, or configuring the clock divisor for CPU_CLK
    // and AHB_CLK ..."
    clk_ll_bus_update();
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_apb_clk_impl(_clocks: &mut ClockTree, new_config: ApbClkConfig) {
    PCR::regs()
        .apb_freq_conf()
        .modify(|_, w| unsafe { w.apb_div_num().bits(new_config.value() as u8) });
}

// XTAL_D2_CLK

fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// LP_FAST_CLK

fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpFastClkConfig>,
    new_selector: LpFastClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.fast_clk_sel().bits(match new_selector {
            LpFastClkConfig::RcFastClk => 0,
            LpFastClkConfig::XtalD2Clk => 1,
            LpFastClkConfig::PllLpClk => 2,
        })
    });
}

// LP_SLOW_CLK

fn enable_lp_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpSlowClkConfig>,
    new_selector: LpSlowClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.slow_clk_sel().bits(match new_selector {
            LpSlowClkConfig::RcSlow => 0,
            LpSlowClkConfig::Xtal32kClk => 1,
            LpSlowClkConfig::OscSlow => 2,
        })
    });
}

// MCPWM0_FUNCTION_CLOCK

fn enable_mcpwm0_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pwm_clk_conf()
        .modify(|_, w| w.pwm_clkm_en().bit(en));
}

fn configure_mcpwm0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Mcpwm0FunctionClockConfig>,
    new_selector: Mcpwm0FunctionClockConfig,
) {
    PCR::regs().pwm_clk_conf().modify(|_, w| unsafe {
        w.pwm_clkm_sel().bits(match new_selector {
            Mcpwm0FunctionClockConfig::XtalClk => 0,
            Mcpwm0FunctionClockConfig::RcFastClk => 1,
            Mcpwm0FunctionClockConfig::PllF96m => 2,
        })
    });
}

// TIMG0_FUNCTION_CLOCK

fn enable_timg0_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup0_timer_clk_conf()
        .modify(|_, w| w.tg0_timer_clk_en().bit(en));
}

fn configure_timg0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0FunctionClockConfig>,
    new_selector: Timg0FunctionClockConfig,
) {
    // TODO: add variants to PAC
    PCR::regs()
        .timergroup0_timer_clk_conf()
        .modify(|_, w| unsafe {
            w.tg0_timer_clk_sel().bits(match new_selector {
                Timg0FunctionClockConfig::XtalClk => 0,
                Timg0FunctionClockConfig::RcFastClk => 1,
                Timg0FunctionClockConfig::PllF48m => 2,
            })
        });
}

// TIMG0_CALIBRATION_CLOCK

fn enable_timg0_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, calibration clocks can only be selected. They are gated by the CALI_START bit,
    // which is managed by the calibration process.
}

fn configure_timg0_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0CalibrationClockConfig>,
    new_selector: Timg0CalibrationClockConfig,
) {
    TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_clk_sel().bits(match new_selector {
            Timg0CalibrationClockConfig::RtcSlowClk => 0,
            Timg0CalibrationClockConfig::RcFastClk => 1,
            Timg0CalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

// TIMG0_WDT_CLOCK

fn enable_timg0_wdt_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup0_wdt_clk_conf()
        .modify(|_, w| w.tg0_wdt_clk_en().bit(en));
}

fn configure_timg0_wdt_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0WdtClockConfig>,
    new_selector: Timg0WdtClockConfig,
) {
    PCR::regs()
        .timergroup0_wdt_clk_conf()
        .modify(|_, w| unsafe {
            w.tg0_wdt_clk_sel().bits(match new_selector {
                Timg0WdtClockConfig::XtalClk => 0,
                Timg0WdtClockConfig::RcFastClk => 1,
                Timg0WdtClockConfig::PllF48m => 2,
            })
        });
}

// TIMG1_FUNCTION_CLOCK

fn enable_timg1_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup1_timer_clk_conf()
        .modify(|_, w| w.tg1_timer_clk_en().bit(en));
}

fn configure_timg1_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0FunctionClockConfig>,
    new_selector: Timg0FunctionClockConfig,
) {
    // TODO: add variants to PAC
    PCR::regs()
        .timergroup1_timer_clk_conf()
        .modify(|_, w| unsafe {
            w.tg1_timer_clk_sel().bits(match new_selector {
                Timg0FunctionClockConfig::XtalClk => 0,
                Timg0FunctionClockConfig::RcFastClk => 1,
                Timg0FunctionClockConfig::PllF48m => 2,
            })
        });
}

// TIMG1_CALIBRATION_CLOCK

fn enable_timg1_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, calibration clocks can only be selected. They are gated by the CALI_START bit,
    // which is managed by the calibration process.
}

fn configure_timg1_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0CalibrationClockConfig>,
    new_selector: Timg0CalibrationClockConfig,
) {
    TIMG1::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_clk_sel().bits(match new_selector {
            Timg0CalibrationClockConfig::RtcSlowClk => 0,
            Timg0CalibrationClockConfig::RcFastClk => 1,
            Timg0CalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

// TIMG1_WDT_CLOCK

fn enable_timg1_wdt_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup1_wdt_clk_conf()
        .modify(|_, w| w.tg1_wdt_clk_en().bit(en));
}

fn configure_timg1_wdt_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0WdtClockConfig>,
    new_selector: Timg0WdtClockConfig,
) {
    PCR::regs()
        .timergroup1_wdt_clk_conf()
        .modify(|_, w| unsafe {
            w.tg1_wdt_clk_sel().bits(match new_selector {
                Timg0WdtClockConfig::XtalClk => 0,
                Timg0WdtClockConfig::RcFastClk => 1,
                Timg0WdtClockConfig::PllF48m => 2,
            })
        });
}

//! Clock tree definitions and implementations for ESP32-C6.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (136k RC_SLOW and 17.5M RC_FAST) are not calibrated here, this system
//!   can only give a rough estimate of their frequency. They can be calibrated separately using a
//!   known crystal frequency.
//! - Some of the SOC capabilities are not implemented: I2S external pad clock source, external 32k
//!   oscillator, LP_DYN_FAST_CLK, APB_DECREASE_DIV (which seems unnecessary to model),
//!   PCR_CPU_HS_120M_FORCE.
#![allow(dead_code, reason = "Some of this is bound to be unused")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use crate::peripherals::{LP_AON_CLKRST, PMU, SYSTEM};

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
    /// 360 MHz CPU clock
    #[default]
    _360MHz  = 360,
}

impl CpuClock {
    const PRESET_360: ClockConfig = ClockConfig {
        xtal_clk: None,
        apb_clk: Some(ApbClkConfig::new(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        ..Default::default()
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_360MHz => CpuClock::PRESET_360,
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
            v if v == CpuClock::PRESET_360 => Some(CpuClock::_360MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(mut self) {
        if self.xtal_clk.is_none() {
            self.xtal_clk = Some(XtalClkConfig::_40);
        }
        self.apply();
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, _config: XtalClkConfig) {
    // The stored configuration affects PLL settings instead.
}

// CPLL_CLK

fn enable_cpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_cpll_clk_impl(_clocks: &mut ClockTree, _config: CpllClkConfig) {
    todo!()
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    HP_SYS_CLKRST::regs().hp_rst_en1().modify(|_,w| w.rst_en_uart0_core().bit(en))
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_fosc_clk().bit(en));
    // TODO: Should the digital clock gate be a different clock node?
    LP_AON_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_fosc().bit(en));
    crate::rom::ets_delay_us(5);
}

// MPLL_CLK

fn enable_mpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_mpll_clk_impl(_clocks: &mut ClockTree, _config: MpllClkConfig) {
    todo!()
}

// SPLL_CLK

fn enable_spll_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_spll_clk_impl(_clocks: &mut ClockTree, _config: SpllClkConfig) {
    todo!()
}

// XTAL32K_CLK

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LP_AON_CLKRST::regs().xtal32k().write(|w| unsafe {
        w.dac_xtal32k().bits(3);
        w.dres_xtal32k().bits(3);
        w.dgm_xtal32k().bits(3);
        w.dbuf_xtal32k().bit(true)
    });

    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_xtal32k().bit(en));

    // Enable for digital part
    // TODO: Should the digital clock gate be a different clock node?
    LP_AON_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_xtal32k().bit(en));
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        // SCK_DCAP value controls tuning of 136k clock. The higher the value of DCAP, the lower the
        // frequency. There is no separate enable bit, just make sure the calibration value is set.
        // const RTC_CNTL_SCK_DCAP_DEFAULT: u8 = 128;
        // crate::soc::regi2c::I2C_DIG_REG_SCK_DCAP.write_reg(RTC_CNTL_SCK_DCAP_DEFAULT);
    }

    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_rc32k().bit(en));

    // Enable for digital part
    LP_AON_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_osc32k().bit(en));
}

// OSC_SLOW_CLK

fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // TODO:
    // gpio_ll_input_enable(&GPIO, SOC_EXT_OSC_SLOW_GPIO_NUM);
    // REG_SET_BIT(LP_AON_GPIO_HOLD0_REG, BIT(SOC_EXT_OSC_SLOW_GPIO_NUM));
    todo!();

    // No need to configure anything else for OSC_SLOW_CLK
}

// PLL_LP_CLK

fn enable_pll_lp_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_pll_lp_clk_impl(_clocks: &mut ClockTree, _config: PllLpClkConfig) {
    todo!()
}

// ROOT_CLK

fn configure_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<RootClkConfig>,
    _new_selector: RootClkConfig,
) {
    todo!()
}

// CPU_CLK

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_cpu_clk_impl(_clocks: &mut ClockTree, _new_config: CpuClkConfig) {
    todo!()
}

// MEM_CLK

fn enable_mem_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_mem_clk_impl(_clocks: &mut ClockTree, _new_config: MemClkConfig) {
    todo!()
}

// SYS_CLK

fn enable_sys_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_sys_clk_impl(_clocks: &mut ClockTree, _new_config: SysClkConfig) {
    todo!()
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_apb_clk_impl(_clocks: &mut ClockTree, _new_config: ApbClkConfig) {
    todo!()
}

// PLL_F50M_CLK

fn enable_pll_f50m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// PLL_F25M_CLK

fn enable_pll_f25m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// PLL_F240M_CLK

fn enable_pll_f240m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// PLL_F160M_CLK

fn enable_pll_f160m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// PLL_F120M_CLK

fn enable_pll_f120m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// PLL_F80M_CLK

fn enable_pll_f80m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// PLL_F20M_CLK

fn enable_pll_f20m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

// LP_SLOW_CLK

fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpSlowClkConfig>,
    new_selector: LpSlowClkConfig,
) {
    LP_AON_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.slow_clk_sel().bits(match new_selector {
            LpSlowClkConfig::Xtal32kClk => 1,
            LpSlowClkConfig::RcSlow => 0,
            LpSlowClkConfig::OscSlow => 2,
        })
    });
}

// LP_FAST_CLK

fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpFastClkConfig>,
    new_selector: LpFastClkConfig,
) {
    LP_AON_CLKRST::regs().lp_clk_conf().modify(|_, w| {
        w.fast_clk_sel().bit(match new_selector {
            LpFastClkConfig::RcFastClk => false,
            LpFastClkConfig::XtalD2Clk => true,
        })
    });
}

// LP_DYN_SLOW_CLK

fn configure_lp_dyn_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpDynSlowClkConfig>,
    _new_selector: LpDynSlowClkConfig,
) {
    todo!()
}

// LP_DYN_FAST_CLK

fn configure_lp_dyn_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpDynFastClkConfig>,
    _new_selector: LpDynFastClkConfig,
) {
    todo!()
}

// LP_PERI_CLK

fn enable_lp_peri_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    todo!()
}

fn configure_lp_peri_clk_impl(_clocks: &mut ClockTree, _new_config: LpPeriClkConfig) {
    todo!()
}

// XTAL_D2_CLK

fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, the divider is always on.
}
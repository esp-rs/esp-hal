//! Clock tree for ESP32-P4X (chip revision v3.x / eco5).
//!
//! CPLL: 400 MHz (eco5 default), 360 MHz (conservative)
//! SPLL: 480 MHz (peripheral clocks)
//! MPLL: 400 MHz (PSRAM, media)
//! XTAL: 40 MHz
//!
//! Clock hierarchy: XTAL -> CPLL -> CPU_ROOT -> CPU/APB dividers
//! SPLL -> PLL_F240M/160M/120M/80M/20M (peripheral clocks)
#![allow(dead_code, reason = "Clock functions called from generated macro code")]
#![allow(
    missing_docs,
    reason = "Clock-tree types come from a generated macro that does not emit doc strings"
)]
#![allow(
    clippy::enum_variant_names,
    reason = "CPU frequency variant names follow the chip-spec MHz convention"
)]

use crate::peripherals::{HP_SYS_CLKRST, LP_AON_CLKRST};

define_clock_tree_types!();

/// CPU clock frequency presets for ESP32-P4X (eco5 / chip revision v3.x).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum CpuClock {
    /// 400 MHz CPU clock (eco5 / v3.x maximum)
    /// CPLL -> CPU_ROOT -> CPU/1, APB divider /4 = 100MHz
    _400MHz = 400,

    /// 360 MHz CPU clock (conservative)
    _360MHz = 360,

    /// 200 MHz CPU clock (low power)
    /// CPLL -> CPU_ROOT -> CPU/2, APB /2 = 100MHz
    _200MHz = 200,

    /// 100 MHz CPU clock (ultra low power)
    /// CPLL -> CPU_ROOT -> CPU/4, APB /1 = 100MHz
    #[default]
    _100MHz = 100,
}

impl CpuClock {
    // Preset: 400 MHz CPU, 100 MHz APB
    const PRESET_400: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpu_clk: Some(CpuClkConfig::new(0)), // /1 = 400 MHz
        apb_clk: Some(ApbClkConfig::new(3)), // /4 = 100 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };

    const PRESET_360: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpu_clk: Some(CpuClkConfig::new(0)), // /1 = 360 MHz (CPLL at 360)
        apb_clk: Some(ApbClkConfig::new(3)), // /4 = 90 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };

    const PRESET_200: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpu_clk: Some(CpuClkConfig::new(1)), // /2 = 200 MHz
        apb_clk: Some(ApbClkConfig::new(1)), // /2 = 100 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };

    const PRESET_100: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpu_clk: Some(CpuClkConfig::new(3)), // /4 = 100 MHz
        apb_clk: Some(ApbClkConfig::new(0)), // /1 = 100 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_400MHz => CpuClock::PRESET_400,
            CpuClock::_360MHz => CpuClock::PRESET_360,
            CpuClock::_200MHz => CpuClock::PRESET_200,
            CpuClock::_100MHz => CpuClock::PRESET_100,
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
            v if v == CpuClock::PRESET_400 => Some(CpuClock::_400MHz),
            v if v == CpuClock::PRESET_360 => Some(CpuClock::_360MHz),
            v if v == CpuClock::PRESET_200 => Some(CpuClock::_200MHz),
            v if v == CpuClock::PRESET_100 => Some(CpuClock::_100MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(self, clocks: &mut ClockTree) {
        self.apply(clocks);
    }
}

// Clock node implementation functions (called from generated macro)
// These must match the function names that define_clock_tree_types!() expects.

// CPU_ROOT_CLK (mux: XTAL / CPLL / RC_FAST)
fn configure_cpu_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<CpuRootClkConfig>,
    new_selector: CpuRootClkConfig,
) {
    //      LP_AON_CLKRST.hp_clk_ctrl.hp_root_clk_src_sel: 0=XTAL, 1=CPLL, 2=RC_FAST
    let sel = match new_selector {
        CpuRootClkConfig::Xtal => 0,
        CpuRootClkConfig::Cpll => 1,
        CpuRootClkConfig::RcFast => 2,
    };
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| unsafe { w.lp_aonclkrst_hp_root_clk_src_sel().bits(sel) });
}

// CPU_CLK divider
fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuClkConfig>,
    new_config: CpuClkConfig,
) {
    //      HP_SYS_CLKRST.root_clk_ctrl0.cpu_clk_div_num = divider - 1
    let clkrst = crate::peripherals::HP_SYS_CLKRST::regs();
    clkrst.root_clk_ctrl0().modify(|_, w| unsafe {
        w.cpu_clk_div_num().bits(new_config.divisor as u8);
        w.cpu_clk_div_numerator().bits(0);
        w.cpu_clk_div_denominator().bits(0)
    });
    // Trigger divider update
    clkrst
        .root_clk_ctrl0()
        .modify(|_, w| w.soc_clk_div_update().set_bit());
    while clkrst
        .root_clk_ctrl0()
        .read()
        .soc_clk_div_update()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

// APB_CLK divider
fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_apb_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<ApbClkConfig>,
    _new_config: ApbClkConfig,
) {
    // TODO: APB divider register in HP_SYS_CLKRST
    // For now, APB freq is derived from CPU freq via divider
}

// LP_FAST_CLK mux
fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpFastClkConfig>,
    new_selector: LpFastClkConfig,
) {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_lp_clk_conf()
        .modify(|_, w| unsafe {
            w.lp_aonclkrst_fast_clk_sel().bits(match new_selector {
                LpFastClkConfig::RcFast => 0,
                LpFastClkConfig::XtalD2 => 1,
            })
        });
}

// LP_SLOW_CLK mux
fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpSlowClkConfig>,
    new_selector: LpSlowClkConfig,
) {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_lp_clk_conf()
        .modify(|_, w| unsafe {
            w.lp_aonclkrst_slow_clk_sel().bits(match new_selector {
                LpSlowClkConfig::RcSlow => 0,
                LpSlowClkConfig::Xtal32k => 1,
                // LpSlowClkConfig::Rc32k => 2,
                LpSlowClkConfig::OscSlow => 3,
            })
        });
}

// Per-instance clock impl for UART (called on UartInstance enum)

impl UartInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // UART function clock enable is handled by peripheral clock gates in system.rs
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        _new_config: UartFunctionClockConfig,
    ) {
        // TODO: Configure UART clock source selection
        // HP_SYS_CLKRST PERI_CLK_CTRL110-114 for UART0-4
    }

    fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Baud rate generator is always on when UART is enabled
    }

    fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartBaudRateGeneratorConfig>,
        _new_config: UartBaudRateGeneratorConfig,
    ) {
        // Baud rate is configured directly in UART registers, not here
    }
}

// Per-instance clock impl for TIMG

impl TimgInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // TIMG function clock is managed by peripheral clock gates
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgFunctionClockConfig>,
        _new_config: TimgFunctionClockConfig,
    ) {
        // TODO: Configure TIMG clock source
        // HP_SYS_CLKRST PERI_CLK_CTRL20/21
    }

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgWdtClockConfig>,
        _new_config: TimgWdtClockConfig,
    ) {
        // TODO: Configure TIMG WDT clock source
    }
}

// System clock impl functions

// Mux enable stubs (mux nodes need enable functions too)
fn enable_cpu_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_lp_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}

// Source clock enable/disable stubs (PLLs, oscillators)
fn enable_cpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_spll_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_mpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_rc32k_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f20m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f80m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f120m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f160m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f240m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f25m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f50m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {}

// TIMG_CALIBRATION_CLOCK

fn enable_timg_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgCalibrationClockConfig>,
    new_config: TimgCalibrationClockConfig,
) {
    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl21()
        .modify(|_, w| unsafe {
            w.timergrp0_tgrt_clk_src_sel().bits(match new_config {
                TimgCalibrationClockConfig::MpllClk => 0,
                TimgCalibrationClockConfig::SpllClk => 1,
                TimgCalibrationClockConfig::CpllClk => 2,
                TimgCalibrationClockConfig::RcFastClk => 7,
                TimgCalibrationClockConfig::RcSlowClk => 8,
                TimgCalibrationClockConfig::Xtal32kClk => 10,
            })
        });
}

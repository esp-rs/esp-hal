#![allow(dead_code, reason = "Placeholder implementation")]
#![allow(missing_docs, reason = "Placeholder implementation")]
#![allow(unused_variables, reason = "Placeholder implementation")]

define_clock_tree_types!();

/// CPU clock speed options.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum CpuClock {
    /// 160 MHz CPU clock
    #[default]
    _160MHz = 160,
    /// 240 MHz CPU clock
    _240MHz = 240,
}

impl CpuClock {
    const PRESET_160: ClockConfig = ClockConfig {
        bbpll_clk: None,
        cpll_clk: None,
        cpu_root_clk: Some(CpuRootClkConfig::PllF240m),
        cpu_clk: Some(CpuClkConfig::new(1)),
        ahb_clk: Some(AhbClkConfig::new(2)),
        apb_clk: Some(ApbClkConfig::new(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
    const PRESET_240: ClockConfig = ClockConfig {
        bbpll_clk: None,
        cpll_clk: None,
        cpu_root_clk: Some(CpuRootClkConfig::PllF240m),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(2)),
        apb_clk: Some(ApbClkConfig::new(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_160MHz => CpuClock::PRESET_160,
            CpuClock::_240MHz => CpuClock::PRESET_240,
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
            v if v == CpuClock::PRESET_160 => Some(CpuClock::_160MHz),
            v if v == CpuClock::PRESET_240 => Some(CpuClock::_240MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(self, clocks: &mut ClockTree) {
        self.apply(clocks);
    }
}

fn enable_bbpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_bbpll_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<BbpllClkConfig>,
    _new: BbpllClkConfig,
) {
}

fn enable_cpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_cpll_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<CpllClkConfig>,
    _new: CpllClkConfig,
) {
}

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}

fn enable_pll_f20m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f40m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f80m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f120m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f160m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f240m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {}

fn enable_cpu_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_cpu_root_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<CpuRootClkConfig>,
    _new: CpuRootClkConfig,
) {
}

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_cpu_clk_impl(_clocks: &mut ClockTree, _old: Option<CpuClkConfig>, _new: CpuClkConfig) {
}

fn enable_ahb_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_ahb_clk_impl(_clocks: &mut ClockTree, _old: Option<AhbClkConfig>, _new: AhbClkConfig) {
}

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_apb_clk_impl(_clocks: &mut ClockTree, _old: Option<ApbClkConfig>, _new: ApbClkConfig) {
}

fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<LpFastClkConfig>,
    _new: LpFastClkConfig,
) {
}

fn enable_lp_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<LpSlowClkConfig>,
    _new: LpSlowClkConfig,
) {
}

fn enable_timg_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old: Option<TimgCalibrationClockConfig>,
    _new: TimgCalibrationClockConfig,
) {
}

// ---- Instance method stubs ----

impl SpiInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}
    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<SpiFunctionClockConfig>,
        _new: SpiFunctionClockConfig,
    ) {
    }
}

impl TimgInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}
    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<TimgFunctionClockConfig>,
        _new: TimgFunctionClockConfig,
    ) {
    }

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}
    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<TimgWdtClockConfig>,
        _new: TimgWdtClockConfig,
    ) {
    }
}

impl UartInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}
    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<UartFunctionClockConfig>,
        _new: UartFunctionClockConfig,
    ) {
    }

    fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {}
    fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<UartBaudRateGeneratorConfig>,
        _new: UartBaudRateGeneratorConfig,
    ) {
    }
}

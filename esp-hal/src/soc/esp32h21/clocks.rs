//! Clock tree definitions and implementations for ESP32-H21.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Internal RC oscillators (600 kHz RC_SLOW and 20 MHz RC_FAST) are not calibrated here, this
//!   system can only give a rough estimate of their frequency. They can be calibrated separately
//!   using a known crystal frequency.
#![allow(dead_code, reason = "Some of this is bound to be unused")]
#![allow(missing_docs, reason = "Experimental")]

use crate::{
    peripherals::{I2C_ANA_MST, LP_CLKRST, PCR, PMU, TIMG0},
    soc::{regi2c, update_bus_clocks, with_single_bus_clock_update},
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
    /// 32 MHz CPU clock, driven by the crystal
    _32MHz = 32,

    /// 48 MHz CPU clock, driven by the BBPLL
    _48MHz = 48,

    /// 64 MHz CPU clock, driven by the crystal frequency doubler
    _64MHz = 64,

    /// 96 MHz CPU clock, driven by the BBPLL
    #[default]
    _96MHz = 96,
}

// AHB_CLK may not exceed 32 MHz and CPU_CLK must be an integer multiple of it, see
// rtc_clk_cpu_freq_to_pll_mhz() and rtc_clk_cpu_freq_to_xtal_x2().
impl CpuClock {
    const PRESET_32: ClockConfig = ClockConfig {
        hp_root_clk: Some(HpRootClkConfig::Xtal),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(0)),
        apb_clk: Some(ApbClkConfig::new(ApbClkDivisor::_0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
        iomux_function_clock: Some(IomuxFunctionClockConfig::PllF48m),
    };
    const PRESET_48: ClockConfig = ClockConfig {
        hp_root_clk: Some(HpRootClkConfig::Pll),
        cpu_clk: Some(CpuClkConfig::new(1)),
        ahb_clk: Some(AhbClkConfig::new(3)),
        apb_clk: Some(ApbClkConfig::new(ApbClkDivisor::_0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
        iomux_function_clock: Some(IomuxFunctionClockConfig::PllF48m),
    };
    const PRESET_64: ClockConfig = ClockConfig {
        hp_root_clk: Some(HpRootClkConfig::XtalX2),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(1)),
        apb_clk: Some(ApbClkConfig::new(ApbClkDivisor::_0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
        iomux_function_clock: Some(IomuxFunctionClockConfig::PllF48m),
    };
    const PRESET_96: ClockConfig = ClockConfig {
        hp_root_clk: Some(HpRootClkConfig::Pll),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(2)),
        apb_clk: Some(ApbClkConfig::new(ApbClkDivisor::_0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
        iomux_function_clock: Some(IomuxFunctionClockConfig::PllF48m),
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_32MHz => CpuClock::PRESET_32,
            CpuClock::_48MHz => CpuClock::PRESET_48,
            CpuClock::_64MHz => CpuClock::PRESET_64,
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
            v if v == CpuClock::PRESET_32 => Some(CpuClock::_32MHz),
            v if v == CpuClock::PRESET_48 => Some(CpuClock::_48MHz),
            v if v == CpuClock::PRESET_64 => Some(CpuClock::_64MHz),
            v if v == CpuClock::PRESET_96 => Some(CpuClock::_96MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(self, clocks: &mut ClockTree) {
        with_single_bus_clock_update(|| self.apply(clocks));
    }
}

// PLL_F96M_CLK

fn enable_pll_f96m_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
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
    }

    PMU::regs().hp_active_hp_ck_power().modify(|_, w| {
        w.hp_active_xpd_bbpll().bit(en);
        w.hp_active_xpd_bbpll_i2c().bit(en)
    });

    if en {
        configure_bbpll();
    }
}

/// The analog part of the BBPLL, see rtc_clk_bbpll_configure() and clk_ll_bbpll_set_config().
///
/// The output frequency is fixed to 96 MHz, so there is nothing to configure on the digital side.
fn configure_bbpll() {
    // BBPLL CALIBRATION START
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    regi2c::I2C_BBPLL_OC_REF_DIV.write_field(8);
    regi2c::I2C_BBPLL_OC_DIV.write_field(24);
    regi2c::I2C_BBPLL_OC_DHREF_SEL.write_field(3);
    regi2c::I2C_BBPLL_OC_DLREF_SEL.write_field(1);

    // WAIT CALIBRATION DONE
    while I2C_ANA_MST::regs()
        .ana_conf0()
        .read()
        .cal_done()
        .bit_is_clear()
    {}

    // Wait for the calibration to truly stop.
    crate::rom::ets_delay_us(10);

    // BBPLL CALIBRATION STOP
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_low().clear_bit();
        w.bbpll_stop_force_high().set_bit()
    });
}

// PLL_F48M_CLK

fn enable_pll_f48m_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_48m_clk_en().bit(en));
}

// IOMUX_FUNCTION_CLOCK

fn configure_iomux_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<IomuxFunctionClockConfig>,
    new_config: IomuxFunctionClockConfig,
) {
    PCR::regs().iomux_clk_conf().modify(|_, w| unsafe {
        w.iomux_func_clk_sel().bits(match new_config {
            IomuxFunctionClockConfig::XtalClk => 0,
            IomuxFunctionClockConfig::RcFastClk => 1,
            IomuxFunctionClockConfig::PllF48m => 2,
        });
        w.iomux_func_clk_en().set_bit()
    });
}

// XTAL_X2_CLK

fn enable_xtal_x2_clk_impl(_clocks: &mut ClockTree, en: bool) {
    // The frequency doubler is a separate circuit, it does not depend on the BBPLL.
    if en {
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
            w.tie_low_xpd_xtalx2().clear_bit();
            w.tie_low_global_xtalx2_icg().clear_bit()
        });
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
            w.tie_high_xtalx2().set_bit();
            w.tie_high_global_xtalx2_icg().set_bit()
        });
    } else {
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
            w.tie_high_xtalx2().clear_bit();
            w.tie_high_global_xtalx2_icg().clear_bit()
        });
        PMU::regs().imm_hp_ck_power().modify(|_, w| {
            w.tie_low_xpd_xtalx2().set_bit();
            w.tie_low_global_xtalx2_icg().set_bit()
        });
    }

    PMU::regs()
        .hp_active_xtal()
        .modify(|_, w| w.hp_active_xpd_xtalx2().bit(en));
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_fosc_clk().bit(en));
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_fosc().bit(en));
}

// XTAL32K_CLK

#[cfg(use_xtal32k)]
fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_xtal32k().bit(en));
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_xtal32k().bit(en));
}

// OSC_SLOW_CLK

fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_osc32k().bit(en));
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_sosc().bit(en));
}

// RC_SLOW_D4_CLK

fn enable_rc_slow_d4_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // The divider is fixed in hardware, there is nothing to gate.
}

// HP_ROOT_CLK

fn enable_hp_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_hp_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<HpRootClkConfig>,
    new_config: HpRootClkConfig,
) {
    PCR::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_config {
            HpRootClkConfig::Xtal => 0,
            HpRootClkConfig::Pll => 1,
            HpRootClkConfig::RcFast => 2,
            HpRootClkConfig::XtalX2 => 3,
        })
    });

    update_bus_clocks();
}

// CPU_CLK

fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuClkConfig>,
    new_config: CpuClkConfig,
) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits(new_config.divisor() as u8) });

    update_bus_clocks();
}

// AHB_CLK

fn configure_ahb_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<AhbClkConfig>,
    new_config: AhbClkConfig,
) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_div_num().bits(new_config.divisor() as u8) });

    update_bus_clocks();
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, en: bool) {
    crate::soc::enable_apb_frequency_reduction(!en);
}

fn configure_apb_clk_impl(
    _clocks: &mut ClockTree,
    old_config: Option<ApbClkConfig>,
    new_config: ApbClkConfig,
) {
    PCR::regs()
        .apb_freq_conf()
        .modify(|_, w| unsafe { w.apb_div_num().bits(new_config.divisor() as u8) });

    // No driver uses APB_CLK before the first configuration.
    if old_config.is_none() {
        crate::soc::enable_apb_frequency_reduction(true);
    }

    update_bus_clocks();
}

// XTAL_D2_CLK

fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

// LP_FAST_CLK

fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<LpFastClkConfig>,
    new_config: LpFastClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.fast_clk_sel().bits(match new_config {
            LpFastClkConfig::RcFastClk => 0,
            LpFastClkConfig::XtalD2Clk => 1,
        })
    });
}

// LP_SLOW_CLK

fn enable_lp_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<LpSlowClkConfig>,
    new_config: LpSlowClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.slow_clk_sel().bits(match new_config {
            LpSlowClkConfig::RcSlow => 0,
            #[cfg(use_xtal32k)]
            LpSlowClkConfig::Xtal32k => 1,
            LpSlowClkConfig::OscSlow => 3,
        })
    });
}

// TIMG_CALIBRATION_CLOCK

fn enable_timg_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, calibration clocks can only be selected. They are gated by the CALI_START
    // bit, which is managed by the calibration process.
}

fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgCalibrationClockConfig>,
    new_config: TimgCalibrationClockConfig,
) {
    // RC_FAST reaches the calibration mux through the tick divider, which esp-idf sets to 32
    // (clk_ll_rc_fast_tick_conf()). Device metadata describes the same divider.
    if new_config == TimgCalibrationClockConfig::RcFastDivClk {
        const RC_FAST_DIVIDER: u8 = 32;
        PCR::regs()
            .ctrl_tick_conf()
            .modify(|_, w| unsafe { w.fosc_tick_num().bits(RC_FAST_DIVIDER - 1) });
    }

    // The 32 kHz calibration input has a second mux: 1 selects XTAL32K, 2 selects OSC_SLOW.
    #[cfg(use_xtal32k)]
    if new_config == TimgCalibrationClockConfig::Xtal32kClk {
        PCR::regs()
            .ctrl_32k_conf()
            .modify(|_, w| unsafe { w._32k_sel().bits(1) });
    }

    TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_clk_sel().bits(match new_config {
            TimgCalibrationClockConfig::RcSlowClk => 0,
            TimgCalibrationClockConfig::RcFastDivClk => 1,
            #[cfg(use_xtal32k)]
            TimgCalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

impl TimgInstance {
    // TIMG_FUNCTION_CLOCK

    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        PCR::regs()
            .timergroup(self as usize)
            .timer_clk_conf()
            .modify(|_, w| w.timer_clk_en().bit(en));
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgFunctionClockConfig>,
        new_config: TimgFunctionClockConfig,
    ) {
        PCR::regs()
            .timergroup(self as usize)
            .timer_clk_conf()
            .modify(|_, w| unsafe {
                w.timer_clk_sel().bits(match new_config {
                    TimgFunctionClockConfig::XtalClk => 0,
                    TimgFunctionClockConfig::RcFastClk => 1,
                    TimgFunctionClockConfig::PllF48m => 2,
                })
            });
    }

    // TIMG_WDT_CLOCK

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        PCR::regs()
            .timergroup(self as usize)
            .wdt_clk_conf()
            .modify(|_, w| w.wdt_clk_en().bit(en));
    }

    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgWdtClockConfig>,
        new_config: TimgWdtClockConfig,
    ) {
        PCR::regs()
            .timergroup(self as usize)
            .wdt_clk_conf()
            .modify(|_, w| unsafe {
                w.wdt_clk_sel().bits(match new_config {
                    TimgWdtClockConfig::XtalClk => 0,
                    TimgWdtClockConfig::RcFastClk => 1,
                    TimgWdtClockConfig::PllF48m => 2,
                })
            });
    }
}

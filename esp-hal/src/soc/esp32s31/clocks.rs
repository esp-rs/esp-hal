//! Clock tree implementation for ESP32-S31.
//!
//! CPLL supplies the 160 and 320 MHz CPU configurations. The independent
//! 480 MHz BBPLL supplies the 240 MHz CPU configuration and peripheral taps.
#![allow(dead_code, reason = "Clock functions called from generated macro code")]
#![allow(
    missing_docs,
    reason = "Clock-tree types come from generated macro code"
)]

use esp_rom_sys::rom::ets_update_cpu_frequency_rom;

use crate::{
    pac::HP_ALIVE_SYS,
    peripherals::{HP_SYS_CLKRST, LP_AON_CLK_RST, PMU},
};

define_clock_tree_types!();

fn hp_alive_sys() -> &'static crate::pac::hp_alive_sys::RegisterBlock {
    // SAFETY: Access is serialized by the clock-tree critical section.
    unsafe { &*HP_ALIVE_SYS::PTR }
}

/// CPU clock speed options.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub enum CpuClock {
    /// 160 MHz CPU clock.
    #[default]
    _160MHz = 160,
    /// 240 MHz CPU clock.
    _240MHz = 240,
    /// 320 MHz CPU clock.
    _320MHz = 320,
}

impl CpuClock {
    const PRESET_160: ClockConfig = ClockConfig {
        bbpll_clk: Some(BbpllClkConfig::_480),
        cpll_clk: Some(CpllClkConfig::_320),
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpu_clk: Some(CpuClkConfig::new(1)),
        ahb_clk: Some(AhbClkConfig::new(1)),
        apb_clk: Some(ApbClkConfig::new(1)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
    const PRESET_240: ClockConfig = ClockConfig {
        bbpll_clk: Some(BbpllClkConfig::_480),
        cpll_clk: None,
        cpu_root_clk: Some(CpuRootClkConfig::PllF240m),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(2)),
        apb_clk: Some(ApbClkConfig::new(1)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
    const PRESET_320: ClockConfig = ClockConfig {
        bbpll_clk: Some(BbpllClkConfig::_480),
        cpll_clk: Some(CpllClkConfig::_320),
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(2)),
        apb_clk: Some(ApbClkConfig::new(1)),
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
            CpuClock::_320MHz => CpuClock::PRESET_320,
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
            v if v == CpuClock::PRESET_320 => Some(CpuClock::_320MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(self, clocks: &mut ClockTree) {
        self.apply(clocks);
    }
}

fn update_bus_clocks() {
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .modify(|_, w| w.soc_clk_div_update().set_bit());
    while HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .read()
        .soc_clk_div_update()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

fn enable_bbpll_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        PMU::regs().imm_hp_ck_power_1().modify(|_, w| {
            w.tie_high_xpd_bbpll().set_bit();
            w.tie_high_xpd_bbpll_i2c().set_bit();
            w.tie_high_global_bbpll_icg().set_bit()
        });
    } else {
        PMU::regs().imm_hp_ck_power_1().modify(|_, w| {
            w.tie_low_global_bbpll_icg().set_bit();
            w.tie_low_xpd_bbpll().set_bit();
            w.tie_low_xpd_bbpll_i2c().set_bit()
        });
    }
    hp_alive_sys()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_spll_480m_clk_en().bit(en));
}

fn configure_bbpll_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<BbpllClkConfig>,
    _new: BbpllClkConfig,
) {
    // The S31 BBPLL is fixed at 480 MHz. Program its documented divider taps.
    HP_SYS_CLKRST::regs()
        .ref_20m_ctrl0()
        .modify(|_, w| unsafe { w.ref_20m_clk_div_num().bits(23) });
    HP_SYS_CLKRST::regs()
        .ref_80m_ctrl0()
        .modify(|_, w| unsafe { w.ref_80m_clk_div_num().bits(5) });
    HP_SYS_CLKRST::regs()
        .ref_120m_ctrl0()
        .modify(|_, w| unsafe { w.ref_120m_clk_div_num().bits(3) });
    HP_SYS_CLKRST::regs()
        .ref_160m_ctrl0()
        .modify(|_, w| unsafe { w.ref_160m_clk_div_num().bits(2) });
    HP_SYS_CLKRST::regs()
        .ref_240m_ctrl0()
        .modify(|_, w| unsafe { w.ref_240m_clk_div_num().bits(1) });
}

fn enable_cpll_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        PMU::regs().imm_hp_ck_power_1().modify(|_, w| {
            w.tie_high_xpd_pll().set_bit();
            w.tie_high_xpd_pll_i2c().set_bit();
            w.tie_high_global_pll_icg().set_bit()
        });
    } else {
        PMU::regs().imm_hp_ck_power_1().modify(|_, w| {
            w.tie_low_global_pll_icg().set_bit();
            w.tie_low_xpd_pll().set_bit();
            w.tie_low_xpd_pll_i2c().set_bit()
        });
    }
    hp_alive_sys()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_cpll_300m_clk_en().bit(en));
}

fn configure_cpll_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<CpllClkConfig>,
    _new: CpllClkConfig,
) {
    // CPLL = XTAL * fb_div / ref_div = 40 MHz * 8 / 1.
    LP_AON_CLK_RST::regs().cpll_div().modify(|_, w| unsafe {
        w.cpll_fb_div().bits(8);
        w.cpll_ref_div().bits(1)
    });

    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().clear_bit());
    while HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .cpu_pll_cal_end()
        .bit_is_clear()
    {
        core::hint::spin_loop();
    }
    crate::rom::ets_delay_us(10);
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().set_bit());
}

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_fosc_clk().bit(en));
    hp_alive_sys()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_fosc_20m_clk_en().bit(en));
    if en {
        crate::rom::ets_delay_us(50);
    }
}

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        LP_AON_CLK_RST::regs().xtal32k().modify(|_, w| unsafe {
            w.dac_xtal32k().bits(7);
            w.dres_xtal32k().bits(7);
            w.dgm_xtal32k().bits(7);
            w.dbuf_xtal32k().set_bit()
        });
    }
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_xtal32k().bit(en));
    hp_alive_sys()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_xtal_32k_clk_en().bit(en));
}

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    hp_alive_sys()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_sosc_150k_clk_en().bit(en));
}

macro_rules! pll_gate {
    ($name:ident, $register:ident, $field:ident) => {
        fn $name(_clocks: &mut ClockTree, en: bool) {
            HP_SYS_CLKRST::regs()
                .$register()
                .modify(|_, w| w.$field().bit(en));
        }
    };
}

pll_gate!(enable_pll_f20m_impl, ref_20m_ctrl0, ref_20m_clk_en);
pll_gate!(enable_pll_f80m_impl, ref_80m_ctrl0, ref_80m_clk_en);
pll_gate!(enable_pll_f120m_impl, ref_120m_ctrl0, ref_120m_clk_en);
pll_gate!(enable_pll_f160m_impl, ref_160m_ctrl0, ref_160m_clk_en);
pll_gate!(enable_pll_f240m_impl, ref_240m_ctrl0, ref_240m_clk_en);

fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here
}

fn enable_cpu_root_clk_impl(_clocks: &mut ClockTree, en: bool) {
    hp_alive_sys()
        .hp_clk_ctrl()
        .modify(|_, w| w.hp_root_clk_en().bit(en));
}

fn configure_cpu_root_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<CpuRootClkConfig>,
    new: CpuRootClkConfig,
) {
    HP_SYS_CLKRST::regs().soc_clk_sel().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new {
            CpuRootClkConfig::Xtal => 0,
            CpuRootClkConfig::Cpll => 1,
            CpuRootClkConfig::RcFast => 2,
            CpuRootClkConfig::PllF240m => 3,
        })
    });
    update_bus_clocks();
}

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here
}

fn configure_cpu_clk_impl(_clocks: &mut ClockTree, _old: Option<CpuClkConfig>, new: CpuClkConfig) {
    HP_SYS_CLKRST::regs()
        .cpu_freq_ctrl0()
        .modify(|_, w| unsafe {
            w.cpu_clk_div_num().bits(new.divisor() as u8);
            w.cpu_clk_div_numerator().bits(0);
            w.cpu_clk_div_denominator().bits(0)
        });
    update_bus_clocks();
    // MEM_CLK is a separate CPU branch and is limited to 160 MHz.
    HP_SYS_CLKRST::regs()
        .mem_freq_ctrl0()
        .modify(|_, w| w.mem_clk_div_num().bit(cpu_clk_frequency() > 160_000_000));
    update_bus_clocks();
    ets_update_cpu_frequency_rom(cpu_clk_frequency() / 1_000_000);
}

fn enable_ahb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here
}

fn configure_ahb_clk_impl(_clocks: &mut ClockTree, _old: Option<AhbClkConfig>, new: AhbClkConfig) {
    HP_SYS_CLKRST::regs()
        .sys_freq_ctrl0()
        .modify(|_, w| unsafe {
            w.sys_clk_div_num().bits(new.divisor() as u8);
            w.sys_clk_div_numerator().bits(0);
            w.sys_clk_div_denominator().bits(0)
        });
    update_bus_clocks();
}

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here
}

fn configure_apb_clk_impl(_clocks: &mut ClockTree, _old: Option<ApbClkConfig>, new: ApbClkConfig) {
    HP_SYS_CLKRST::regs()
        .apb_freq_ctrl0()
        .modify(|_, w| unsafe {
            w.apb_clk_div_num().bits(new.divisor() as u8);
            w.apb_clk_div_numerator().bits(0);
            w.apb_clk_div_denominator().bits(0)
        });
    update_bus_clocks();
}

fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here
}

fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<LpFastClkConfig>,
    new: LpFastClkConfig,
) {
    LP_AON_CLK_RST::regs()
        .root_clk_conf()
        .modify(|_, w| unsafe {
            w.fast_clk_sel().bits(match new {
                LpFastClkConfig::RcFast => 0,
                LpFastClkConfig::Xtal => 1,
            })
        });
}

fn enable_lp_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here
}

fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<LpSlowClkConfig>,
    new: LpSlowClkConfig,
) {
    LP_AON_CLK_RST::regs()
        .root_clk_conf()
        .modify(|_, w| unsafe {
            w.slow_clk_sel().bits(match new {
                LpSlowClkConfig::RcSlow => 0,
                LpSlowClkConfig::Xtal32k => 1,
            })
        });
}

fn enable_timg_calibration_clock_impl(_clocks: &mut ClockTree, en: bool) {
    HP_SYS_CLKRST::regs()
        .timergrp0_tgrt_ctrl0()
        .modify(|_, w| w.timergrp0_tgrt_clk_en().bit(en));
}

fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old: Option<TimgCalibrationClockConfig>,
    new: TimgCalibrationClockConfig,
) {
    let (source, divider): (u8, u16) = match new {
        TimgCalibrationClockConfig::RcFastDivClk => (7, 50),
        TimgCalibrationClockConfig::RcSlowClk => (8, 1),
        TimgCalibrationClockConfig::Xtal32kClk => (10, 1),
    };
    HP_SYS_CLKRST::regs()
        .timergrp0_tgrt_ctrl0()
        .modify(|_, w| unsafe {
            w.timergrp0_tgrt_clk_src_sel().bits(source);
            w.timergrp0_tgrt_clk_div_num().bits(divider - 1)
        });
}

impl I2cInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do here
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<I2cFunctionClockConfig>,
        new: I2cFunctionClockConfig,
    ) {
        HP_SYS_CLKRST::regs().i2c0_ctrl0().modify(|_, w| unsafe {
            w.i2c0_clk_src_sel()
                .bit(matches!(new.sclk(), I2cFunctionClockSclk::RcFast));
            w.i2c0_clk_div_num().bits(new.div_num() as u8);
            w.i2c0_clk_div_numerator().bits(0);
            w.i2c0_clk_div_denominator().bits(0)
        });
    }
}

impl SpiInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do here
    }
    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<SpiFunctionClockConfig>,
        _new: SpiFunctionClockConfig,
    ) {
        // TODO: Configure the GPSPI source and divider when SPI support is enabled.
    }
}

impl TimgInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // TODO: Control the selected timer's function-clock gate.
    }
    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<TimgFunctionClockConfig>,
        _new: TimgFunctionClockConfig,
    ) {
        // TODO: Configure the selected timer's function-clock source.
    }

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // TODO: Control the selected timer group's watchdog-clock gate.
    }
    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old: Option<TimgWdtClockConfig>,
        _new: TimgWdtClockConfig,
    ) {
        // TODO: Configure the selected timer group's watchdog-clock source.
    }
}

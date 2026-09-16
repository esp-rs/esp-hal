//! Clock tree definitions and implementations for ESP32-H4.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Internal RC oscillators (32 kHz OSC_SLOW, 600 kHz RC_SLOW and 20 MHz RC_FAST) are not
//!   calibrated here, this system can only give a rough estimate of their frequency. They can be
//!   calibrated separately using a known crystal frequency.
#![allow(dead_code, reason = "Some of this is bound to be unused")]
#![allow(missing_docs, reason = "Experimental")]

use core::sync::atomic::{AtomicBool, Ordering};

use esp_rom_sys::rom::ets_update_cpu_frequency_rom;

use crate::{
    peripherals::{I2C_ANA_MST, LP_CLKRST, PCR, PMU},
    soc::{regi2c, xtal32k},
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

    /// 96 MHz CPU clock, driven by the BBPLL
    #[default]
    _96MHz = 96,
}

impl CpuClock {
    const PRESET_32: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::Xtal),
        cpu_clk: Some(CpuClkConfig::new(CpuClkDivisor::_0)),
        ahb_clk: Some(AhbClkConfig::new(0)),
        apb_clk: Some(ApbClkConfig::new(ApbClkDivisor::_0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(xtal32k::default_lp_slow_clk()),
        timg_calibration_clock: None,
        iomux_function_clock: Some(IomuxFunctionClockConfig::PllF48m),
    };
    const PRESET_96: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::Pll),
        cpu_clk: Some(CpuClkConfig::new(CpuClkDivisor::_0)),
        // AHB_CLK may not exceed 32 MHz, see rtc_clk_cpu_freq_to_pll_mhz().
        ahb_clk: Some(AhbClkConfig::new(2)),
        apb_clk: Some(ApbClkConfig::new(ApbClkDivisor::_0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(xtal32k::default_lp_slow_clk()),
        timg_calibration_clock: None,
        iomux_function_clock: Some(IomuxFunctionClockConfig::PllF48m),
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_32MHz => CpuClock::PRESET_32,
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
            v if v == CpuClock::PRESET_96 => Some(CpuClock::_96MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(mut self, clocks: &mut ClockTree) {
        if self.xtal_clk.is_none() {
            self.xtal_clk = Some(XtalClkConfig::_32);
        }

        // HP_ROOT_CLK and the CPU, AHB and APB dividers share one update signal. Write the
        // complete configuration before latching it, applying each change separately would
        // temporarily overclock the buses.
        BUS_CLOCK_UPDATE_DEFERRED.store(true, Ordering::Relaxed);
        self.apply(clocks);
        BUS_CLOCK_UPDATE_DEFERRED.store(false, Ordering::Relaxed);

        update_bus_clocks();

        // esp_rom_set_cpu_ticks_per_us(), the ROM delay functions need this.
        ets_update_cpu_frequency_rom(cpu_clk_frequency() / 1_000_000);
    }
}

static BUS_CLOCK_UPDATE_DEFERRED: AtomicBool = AtomicBool::new(false);

/// Applies a new `soc_clk_sel`, `cpu_div_num`, `ahb_div_num` or `apb_div_num`.
///
/// The bus may get stuck if the new combination is invalid.
fn update_bus_clocks() {
    if BUS_CLOCK_UPDATE_DEFERRED.load(Ordering::Relaxed) {
        return;
    }

    PCR::regs()
        .bus_clk_update()
        .write(|w| w.bus_clock_update().set_bit());
    while PCR::regs()
        .bus_clk_update()
        .read()
        .bus_clock_update()
        .bit_is_set()
    {}
}

// XTAL_CLK

fn configure_xtal_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<XtalClkConfig>,
    _config: XtalClkConfig,
) {
    // Nothing to do here.
}

// PLL_F96M_CLK

fn enable_pll_f96m_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_high_xpd_bbpll().set_bit();
            w.tie_high_xpd_bbpll_i2c().set_bit()
        });
    } else {
        PMU::regs()
            .imm_hp_ck_power()
            .write(|w| w.tie_low_global_bbpll_icg().set_bit());
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_low_xpd_bbpll().set_bit();
            w.tie_low_xpd_bbpll_i2c().set_bit()
        });
    }

    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_96m_clk_en().bit(en));

    if en {
        configure_bbpll();
    }
}

/// The analog part of the BBPLL, same shape as on the ESP32-H2 but with this chip's dividers.
///
/// See rtc_clk_bbpll_configure() and clk_ll_bbpll_set_config(). The output frequency is fixed to
/// 96 MHz, so there is nothing to configure on the digital side.
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
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_low_xpd_xtalx2().clear_bit();
            w.tie_low_global_xtalx2_icg().clear_bit()
        });
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_high_xtalx2().set_bit();
            w.tie_high_global_xtalx2_icg().set_bit()
        });
    } else {
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_high_xtalx2().clear_bit();
            w.tie_high_global_xtalx2_icg().clear_bit()
        });
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_low_xpd_xtalx2().set_bit();
            w.tie_low_global_xtalx2_icg().set_bit()
        });
    }
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
            HpRootClkConfig::RcFast => 1,
            HpRootClkConfig::XtalX2 => 2,
            HpRootClkConfig::Pll => 3,
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

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_apb_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<ApbClkConfig>,
    new_config: ApbClkConfig,
) {
    PCR::regs()
        .apb_freq_conf()
        .modify(|_, w| unsafe { w.apb_div_num().bits(new_config.divisor() as u8) });

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
    // The mux only selects between two sources, so the selector is a single bit.
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| {
        w.fast_clk_sel().bit(match new_config {
            LpFastClkConfig::RcFastClk => false,
            LpFastClkConfig::XtalD2Clk => true,
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
    // Nothing to do here.
}

fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgCalibrationClockConfig>,
    new_config: TimgCalibrationClockConfig,
) {
    // RC_FAST is only reachable through the secondary mux, behind a divider. esp-idf divides it
    // by 32 (rtc_time.c, CLK_CAL_DIV_VAL) because calibration needs a relatively slow signal;
    // device metadata describes the same divider.
    const RC_FAST_SECURE_CLK_SEL: u8 = 11;
    const RC_FAST_DIVIDER: u8 = 32;

    PCR::regs().timg_cali_clk_conf().modify(|_, w| unsafe {
        match new_config {
            #[cfg(use_xtal32k)]
            TimgCalibrationClockConfig::Xtal32kClk => w.timg_cali_clk_sel().bits(1),
            TimgCalibrationClockConfig::RcSlowClk => w.timg_cali_clk_sel().bits(3),
            TimgCalibrationClockConfig::RcFastDivClk => {
                w.timg_cali_clk_sel().bits(4);
                w.timg_secure_clk_sel().bits(RC_FAST_SECURE_CLK_SEL);
                w.timg_secure_clk_div_num().bits(RC_FAST_DIVIDER - 1)
            }
        }
    });
}

impl TimgInstance {
    // TIMG_FUNCTION_CLOCK

    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        match self {
            TimgInstance::Timg0 => PCR::regs()
                .timergroup0_timer_clk_conf()
                .modify(|_, w| w.tg0_timer_clk_en().bit(en)),
            TimgInstance::Timg1 => PCR::regs()
                .timergroup1_timer_clk_conf()
                .modify(|_, w| w.tg1_timer_clk_en().bit(en)),
        };
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgFunctionClockConfig>,
        new_config: TimgFunctionClockConfig,
    ) {
        let sel = match new_config {
            TimgFunctionClockConfig::XtalClk => 0,
            TimgFunctionClockConfig::RcFastClk => 1,
            TimgFunctionClockConfig::PllF48m => 2,
        };

        match self {
            TimgInstance::Timg0 => PCR::regs()
                .timergroup0_timer_clk_conf()
                .modify(|_, w| unsafe { w.tg0_timer_clk_sel().bits(sel) }),
            TimgInstance::Timg1 => PCR::regs()
                .timergroup1_timer_clk_conf()
                .modify(|_, w| unsafe { w.tg1_timer_clk_sel().bits(sel) }),
        };
    }

    // TIMG_WDT_CLOCK

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        match self {
            TimgInstance::Timg0 => PCR::regs()
                .timergroup0_wdt_clk_conf()
                .modify(|_, w| w.tg0_wdt_clk_en().bit(en)),
            TimgInstance::Timg1 => PCR::regs()
                .timergroup1_wdt_clk_conf()
                .modify(|_, w| w.tg1_wdt_clk_en().bit(en)),
        };
    }

    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgWdtClockConfig>,
        new_config: TimgWdtClockConfig,
    ) {
        // The watchdogs encode their sources differently from the timers, see mwdt_ll.h and
        // timer_ll.h.
        let sel = match new_config {
            TimgWdtClockConfig::XtalClk => 0,
            TimgWdtClockConfig::PllF48m => 1,
            TimgWdtClockConfig::RcFastClk => 2,
        };

        match self {
            TimgInstance::Timg0 => PCR::regs()
                .timergroup0_wdt_clk_conf()
                .modify(|_, w| unsafe { w.tg0_wdt_clk_sel().bits(sel) }),
            TimgInstance::Timg1 => PCR::regs()
                .timergroup1_wdt_clk_conf()
                .modify(|_, w| unsafe { w.tg1_wdt_clk_sel().bits(sel) }),
        };
    }
}

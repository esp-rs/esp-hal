//! Clock tree definitions and implementations for ESP32-C61.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (32K OSC_SLOW, 130k RC_SLOW and 20M RC_FAST) are not calibrated here,
//!   this system can only give a rough estimate of their frequency. They can be calibrated
//!   separately using a known crystal frequency.
//! - Some of the SOC capabilities are not implemented: I2S external pad clock source, external 32k
//!   oscillator, others.
#![allow(dead_code, reason = "Some of this is bound to be unused")]
#![allow(missing_docs, reason = "Experimental")]

use crate::{
    peripherals::{I2C_ANA_MST, LP_CLKRST, MODEM_LPCON, PCR, PMU, UART0, UART1},
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
    /// Default CPU clock
    #[default]
    _80MHz  = 80,

    /// 160 MHz CPU clock
    _160MHz = 160,
}

impl CpuClock {
    const PRESET_80: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::PllF160m),
        cpu_clk: Some(CpuClkConfig::new(1)),
        ahb_clk: Some(AhbClkConfig::new(3)),
        apb_clk: Some(ApbClkConfig::new(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
    const PRESET_160: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::PllF160m),
        cpu_clk: Some(CpuClkConfig::new(0)),
        ahb_clk: Some(AhbClkConfig::new(3)),
        apb_clk: Some(ApbClkConfig::new(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_80MHz => CpuClock::PRESET_80,
            CpuClock::_160MHz => CpuClock::PRESET_160,
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
            v if v == CpuClock::PRESET_80 => Some(CpuClock::_80MHz),
            v if v == CpuClock::PRESET_160 => Some(CpuClock::_160MHz),
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

fn configure_xtal_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<XtalClkConfig>,
    _config: XtalClkConfig,
) {
    // Nothing to do here.
}

fn enable_pll_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_high_xpd_bb_i2c().set_bit();
            w.tie_high_xpd_bbpll().set_bit();
            w.tie_high_xpd_bbpll_i2c().set_bit()
        });
        PMU::regs()
            .imm_hp_ck_power()
            .write(|w| w.tie_high_global_bbpll_icg().set_bit());
    } else {
        PMU::regs()
            .imm_hp_ck_power()
            .write(|w| w.tie_low_global_bbpll_icg().set_bit());
        PMU::regs().imm_hp_ck_power().write(|w| {
            w.tie_low_xpd_bb_i2c().set_bit();
            w.tie_low_xpd_bbpll().set_bit();
            w.tie_low_xpd_bbpll_i2c().set_bit()
        });

        return;
    }

    // Digital part - The target SPLL is fixed to 480MHz, do nothing.

    // Analog part
    let old_clk_conf = MODEM_LPCON::regs().clk_conf().read();
    MODEM_LPCON::regs().clk_conf().write(|w| {
        unsafe { w.bits(old_clk_conf.bits()) };
        w.clk_i2c_mst_en().set_bit()
    });

    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    const DIV7_0: u8 = 8;
    const DIV_REF: u8 = 0; // Do not divide reference clock
    const DCHGP: u8 = 5;
    const DBIAS: u8 = 3;
    const HREF: u8 = 3;
    const LREF: u8 = 1;

    const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
    // const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;
    // const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

    const I2C_BBPLL_LREF: u8 = (DCHGP << I2C_BBPLL_OC_DCHGP_LSB) | DIV_REF;

    regi2c::I2C_BBPLL_OC_REF.write_reg(I2C_BBPLL_LREF);
    regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(DIV7_0);
    regi2c::I2C_BBPLL_OC_DLREF_SEL.write_field(LREF);
    regi2c::I2C_BBPLL_OC_DHREF_SEL.write_field(HREF);
    regi2c::I2C_BBPLL_OC_VCO_DBIAS.write_field(DBIAS);

    // WAIT CALIBRATION DONE
    while I2C_ANA_MST::regs()
        .ana_conf0()
        .read()
        .cal_done()
        .bit_is_clear()
    {}
    crate::rom::ets_delay_us(10);

    // BBPLL CALIBRATION STOP
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().set_bit();
        w.bbpll_stop_force_low().clear_bit()
    });

    MODEM_LPCON::regs()
        .clk_conf()
        .write(|w| unsafe { w.bits(old_clk_conf.bits()) });
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_fosc().bit(en));
}

// XTAL32K_CLK

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
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

// PLL_F20M

fn enable_pll_f20m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_20m_clk_en().bit(en));
}

// PLL_F40M

fn enable_pll_f40m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_40m_clk_en().bit(en));
}

// PLL_F48M

fn enable_pll_f48m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_48m_clk_en().bit(en));
}

// PLL_F60M

fn enable_pll_f60m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_60m_clk_en().bit(en));
}

// PLL_F80M

fn enable_pll_f80m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_80m_clk_en().bit(en));
}

// PLL_F120M

fn enable_pll_f120m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_120m_clk_en().bit(en));
}

// PLL_F160M

fn enable_pll_f160m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_160m_clk_en().bit(en));
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
    // TODO: Limit AHB to ensure it's running at <= XTAL_CLK && CPU_CLK must be an integer multiple
    // of AHB
    PCR::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_config {
            HpRootClkConfig::Xtal => 0,
            HpRootClkConfig::RcFast => 1,
            HpRootClkConfig::PllF160m => 2,
        })
    });
}

// CPU_CLK

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuClkConfig>,
    new_config: CpuClkConfig,
) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits(new_config.divisor() as u8) });

    PCR::regs()
        .bus_clk_update()
        .write(|w| w.bus_clock_update().set_bit());
}

// AHB_CLK

fn enable_ahb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_ahb_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<AhbClkConfig>,
    new_config: AhbClkConfig,
) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_div_num().bits(new_config.divisor() as u8) });

    PCR::regs()
        .bus_clk_update()
        .write(|w| w.bus_clock_update().set_bit());
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
            LpFastClkConfig::RcFast => 0,
            LpFastClkConfig::XtalD2 => 1,
            LpFastClkConfig::Xtal => 2,
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
            LpSlowClkConfig::Xtal32k => 1,
            // LpSlowClkConfig::Ext32k => 2,
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
    PCR::regs().ctrl_32k_conf().modify(|_, w| unsafe {
        w._32k_sel().bits(match new_config {
            TimgCalibrationClockConfig::OscSlowClk => 0,
            TimgCalibrationClockConfig::Xtal32kClk => 1,
            // TimgCalibrationClockConfig::Ext32kClk => 2,
            TimgCalibrationClockConfig::RcSlowClk => 3,
            TimgCalibrationClockConfig::RcFastDivClk => 4,
        })
    });
}

impl TimgInstance {
    // TIMG_FUNCTION_CLOCK

    // TIMG0_FUNCTION_CLOCK

    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let timg = match self {
            TimgInstance::Timg0 => 0,
            TimgInstance::Timg1 => 1,
        };
        PCR::regs()
            .timergroup(timg)
            .timer_clk_conf()
            .modify(|_, w| w.timer_clk_en().bit(en));
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgFunctionClockConfig>,
        new_config: TimgFunctionClockConfig,
    ) {
        // TODO: add variants to PAC
        let timg = match self {
            TimgInstance::Timg0 => 0,
            TimgInstance::Timg1 => 1,
        };
        PCR::regs()
            .timergroup(timg)
            .timer_clk_conf()
            .modify(|_, w| unsafe {
                w.timer_clk_sel().bits(match new_config {
                    TimgFunctionClockConfig::XtalClk => 0,
                    TimgFunctionClockConfig::RcFastClk => 1,
                    TimgFunctionClockConfig::PllF80m => 2,
                })
            });
    }

    // TIMG_WDT_CLOCK

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let timg = match self {
            TimgInstance::Timg0 => 0,
            TimgInstance::Timg1 => 1,
        };
        PCR::regs()
            .timergroup(timg)
            .wdt_clk_conf()
            .modify(|_, w| w.wdt_clk_en().bit(en));
    }

    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgWdtClockConfig>,
        new_config: TimgWdtClockConfig,
    ) {
        let timg = match self {
            TimgInstance::Timg0 => 0,
            TimgInstance::Timg1 => 1,
        };
        PCR::regs()
            .timergroup(timg)
            .wdt_clk_conf()
            .modify(|_, w| unsafe {
                w.wdt_clk_sel().bits(match new_config {
                    TimgWdtClockConfig::XtalClk => 0,
                    TimgWdtClockConfig::RcFastClk => 1,
                    TimgWdtClockConfig::PllF80m => 2,
                })
            });
    }
}

impl UartInstance {
    // UART_FUNCTION_CLOCK

    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let uart = match self {
            UartInstance::Uart0 => {
                // Disabling this prevents the device from booting
                // TODO: https://github.com/esp-rs/esp-hal/issues/4952
                return;
            }
            UartInstance::Uart1 => 1,
        };
        PCR::regs()
            .uart(uart)
            .clk_conf()
            .modify(|_, w| w.sclk_en().bit(en));
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        new_config: UartFunctionClockConfig,
    ) {
        PCR::regs()
            .uart(match self {
                UartInstance::Uart0 => 0,
                UartInstance::Uart1 => 1,
            })
            .clk_conf()
            .modify(|_, w| unsafe {
                w.sclk_sel().bits(match new_config.sclk {
                    UartFunctionClockSclk::Xtal => 0,
                    UartFunctionClockSclk::RcFast => 1,
                    UartFunctionClockSclk::PllF80m => 2,
                });
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.sclk_div_num().bits(new_config.div_num as _);
                w
            });
    }

    // UART_BAUD_RATE_GENERATOR

    fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartBaudRateGeneratorConfig>,
        new_config: UartBaudRateGeneratorConfig,
    ) {
        let regs = match self {
            UartInstance::Uart0 => UART0::regs(),
            UartInstance::Uart1 => UART1::regs(),
        };
        regs.clkdiv().write(|w| unsafe {
            w.clkdiv().bits(new_config.integral as _);
            w.frag().bits(new_config.fractional as _)
        });
    }
}

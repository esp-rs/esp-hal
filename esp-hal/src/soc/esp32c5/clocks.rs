//! Clock tree definitions and implementations for ESP32-C5.
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

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use crate::{
    peripherals::{I2C_ANA_MST, LP_CLKRST, MODEM_LPCON, PCR, PMU},
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
    /// 80 MHz CPU clock
    #[default]
    _80MHz  = 80,

    /// 160 MHz CPU clock
    _160MHz = 160,

    /// 240 MHz CPU clock
    _240MHz = 240,
}

impl CpuClock {
    const PRESET_80: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::PllF160m),
        cpu_clk: Some(CpuClkConfig(1)),
        ahb_clk: Some(AhbClkConfig(3)), // 40MHz - cannot exceed XTAL_CLK
        apb_clk: Some(ApbClkConfig(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        crypto_clk: Some(CryptoClkConfig::PllF480m),
        timg_calibration_clock: None,
    };
    const PRESET_160: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::PllF160m),
        cpu_clk: Some(CpuClkConfig(0)),
        ahb_clk: Some(AhbClkConfig(3)), // 40MHz - cannot exceed XTAL_CLK
        apb_clk: Some(ApbClkConfig(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        crypto_clk: Some(CryptoClkConfig::PllF480m),
        timg_calibration_clock: None,
    };
    const PRESET_240: ClockConfig = ClockConfig {
        xtal_clk: None,
        hp_root_clk: Some(HpRootClkConfig::PllF240m),
        cpu_clk: Some(CpuClkConfig(0)),
        ahb_clk: Some(AhbClkConfig(5)), // 40MHz - cannot exceed XTAL_CLK
        apb_clk: Some(ApbClkConfig(0)),
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        crypto_clk: Some(CryptoClkConfig::PllF480m),
        timg_calibration_clock: None,
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_80MHz => CpuClock::PRESET_80,
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
            v if v == CpuClock::PRESET_80 => Some(CpuClock::_80MHz),
            v if v == CpuClock::PRESET_160 => Some(CpuClock::_160MHz),
            v if v == CpuClock::PRESET_240 => Some(CpuClock::_240MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(mut self) {
        // FIXME: we ignore user XTAL configuration, but we shouldn't offer it in the first place.
        // PCR_CLK_XTAL_FREQ updates its value based on EFUSE_XTAL_48M_SEL.
        self.xtal_clk = if PCR::regs().sysclk_conf().read().clk_xtal_freq().bits() == 40 {
            Some(XtalClkConfig::_40)
        } else {
            Some(XtalClkConfig::_48)
        };

        self.apply();
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, _config: XtalClkConfig) {
    // Nothing to do here.
}

// PLL_CLK

fn enable_pll_clk_impl(clocks: &mut ClockTree, en: bool) {
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

    // Digital part - nothing to do here, PLL always runs at 480MHz

    // Analog part
    // TODO: reference count I2C_ANA_MST clock (also applies to other chips)
    let old_clk_conf = MODEM_LPCON::regs().clk_conf().read();
    MODEM_LPCON::regs().clk_conf().write(|w| {
        unsafe { w.bits(old_clk_conf.bits()) };
        w.clk_i2c_mst_en().set_bit()
    });

    // BBPLL CALIBRATION START
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    let div7_0: u8;
    let dr1: u8;
    let dr3: u8;
    match unwrap!(clocks.xtal_clk()) {
        XtalClkConfig::_40 => {
            div7_0 = 12;
            dr1 = 0;
            dr3 = 0;
        }
        XtalClkConfig::_48 => {
            div7_0 = 10;
            dr1 = 1;
            dr3 = 1;
        }
    }

    const DIV_REF: u8 = 1; // Do not divide reference clock
    const DCHGP: u8 = 5;
    const DBIAS: u8 = 3;
    const HREF: u8 = 3;
    const LREF: u8 = 1;

    const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
    const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;
    const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

    const I2C_BBPLL_LREF: u8 = (DCHGP << I2C_BBPLL_OC_DCHGP_LSB) | DIV_REF;

    regi2c::I2C_BBPLL_OC_REF.write_reg(I2C_BBPLL_LREF);
    regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(div7_0);
    regi2c::I2C_BBPLL_OC_DR1.write_field(dr1);
    regi2c::I2C_BBPLL_OC_DR3.write_field(dr3);
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

// PLL_F12M

fn enable_pll_f12m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_12m_clk_en().bit(en));
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

// PLL_F240M

fn enable_pll_f240m_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .pll_div_clk_en()
        .modify(|_, w| w.pll_240m_clk_en().bit(en));
}

// HP_ROOT_CLK

fn enable_hp_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_hp_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<HpRootClkConfig>,
    new_selector: HpRootClkConfig,
) {
    // TODO: Limit AHB to ensure it's running at <= XTAL_CLK && CPU_CLK must be an integer multiple
    // of AHB
    PCR::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_selector {
            HpRootClkConfig::Xtal => 0,
            HpRootClkConfig::RcFast => 1,
            HpRootClkConfig::PllF160m => 2,
            HpRootClkConfig::PllF240m => 3,
        })
    });
}

// CPU_CLK

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_cpu_clk_impl(_clocks: &mut ClockTree, new_config: CpuClkConfig) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits(new_config.value() as u8) });

    PCR::regs()
        .bus_clk_update()
        .write(|w| w.bus_clock_update().set_bit());
}

// AHB_CLK

fn enable_ahb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_ahb_clk_impl(_clocks: &mut ClockTree, new_config: AhbClkConfig) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_div_num().bits(new_config.value() as u8) });

    PCR::regs()
        .bus_clk_update()
        .write(|w| w.bus_clock_update().set_bit());
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_apb_clk_impl(_clocks: &mut ClockTree, new_config: ApbClkConfig) {
    PCR::regs()
        .apb_freq_conf()
        .modify(|_, w| unsafe { w.apb_div_num().bits(new_config.value() as u8) });
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
    _old_selector: Option<LpFastClkConfig>,
    new_selector: LpFastClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.fast_clk_sel().bits(match new_selector {
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
    _old_selector: Option<LpSlowClkConfig>,
    new_selector: LpSlowClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.slow_clk_sel().bits(match new_selector {
            LpSlowClkConfig::RcSlow => 0,
            LpSlowClkConfig::Xtal32k => 1,
            // LpSlowClkConfig::Ext32k => 2,
            LpSlowClkConfig::OscSlow => 3,
        })
    });
}

// CRYPTO_CLK

fn enable_crypto_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_crypto_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<CryptoClkConfig>,
    new_selector: CryptoClkConfig,
) {
    PCR::regs().sec_conf().modify(|_, w| unsafe {
        w.sec_clk_sel().bits(match new_selector {
            CryptoClkConfig::Xtal => 0,
            CryptoClkConfig::Fosc => 1,
            CryptoClkConfig::PllF480m => 2,
        })
    });
}

// TIMG_CALIBRATION_CLOCK

fn enable_timg_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<TimgCalibrationClockConfig>,
    new_selector: TimgCalibrationClockConfig,
) {
    PCR::regs().ctrl_32k_conf().modify(|_, w| unsafe {
        w._32k_sel().bits(match new_selector {
            TimgCalibrationClockConfig::OscSlowClk => 0,
            TimgCalibrationClockConfig::Xtal32kClk => 1,
            // TimgCalibrationClockConfig::Ext32kClk => 2,
            TimgCalibrationClockConfig::RcSlowClk => 3,
            TimgCalibrationClockConfig::RcFastDivClk => 4,
        })
    });
}

// RMT_SCLK

fn enable_rmt_sclk_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs().rmt_pd_ctrl().modify(|_, w| {
        w.rmt_mem_force_pu().bit(en);
        w.rmt_mem_force_pd().bit(!en)
    });

    PCR::regs()
        .rmt_sclk_conf()
        .modify(|_, w| w.sclk_en().bit(en));
}

fn configure_rmt_sclk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<RmtSclkConfig>,
    new_selector: RmtSclkConfig,
) {
    PCR::regs().rmt_sclk_conf().modify(|_, w| unsafe {
        w.sclk_sel().bits(match new_selector {
            RmtSclkConfig::XtalClk => 0,
            RmtSclkConfig::RcFastClk => 1,
            RmtSclkConfig::PllF80m => 2,
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
                Timg0FunctionClockConfig::PllF80m => 2,
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
                Timg0WdtClockConfig::PllF80m => 2,
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
                Timg0FunctionClockConfig::PllF80m => 2,
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
                Timg0WdtClockConfig::PllF80m => 2,
            })
        });
}

// UART0_FUNCTION_CLOCK

fn enable_uart0_function_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Disabling this prevents the device from booting
    // PCR::regs()
    //    .uart(0)
    //    .clk_conf()
    //    .modify(|_, w| w.sclk_en().bit(en));
}

fn configure_uart0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Uart0FunctionClockConfig>,
    new_selector: Uart0FunctionClockConfig,
) {
    PCR::regs().uart(0).clk_conf().modify(|_, w| unsafe {
        w.sclk_sel().bits(match new_selector {
            Uart0FunctionClockConfig::Xtal => 0,
            Uart0FunctionClockConfig::RcFast => 1,
            Uart0FunctionClockConfig::PllF80m => 2,
        })
    });
}

// UART1_FUNCTION_CLOCK

fn enable_uart1_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .uart(1)
        .clk_conf()
        .modify(|_, w| w.sclk_en().bit(en));
}

fn configure_uart1_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Uart0FunctionClockConfig>,
    new_selector: Uart0FunctionClockConfig,
) {
    PCR::regs().uart(1).clk_conf().modify(|_, w| unsafe {
        w.sclk_sel().bits(match new_selector {
            Uart0FunctionClockConfig::Xtal => 0,
            Uart0FunctionClockConfig::RcFast => 1,
            Uart0FunctionClockConfig::PllF80m => 2,
        })
    });
}

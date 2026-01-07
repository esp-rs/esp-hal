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
    /// 80 MHz CPU clock
    #[default]
    _80MHz  = 80,

    /// 160 MHz CPU clock
    _160MHz = 160,
}

impl CpuClock {
    const PRESET_80: ClockConfig = ClockConfig {
        xtal_clk: None,
        soc_root_clk: Some(SocRootClkConfig::Pll),
        cpu_hs_div: Some(CpuHsDivConfig::_1),
        cpu_ls_div: None, // Unused when root clock is PLL
        ahb_hs_div: Some(AhbHsDivConfig::_3),
        ahb_ls_div: None, // Unused when root clock is PLL
        // Configures 80MHz MSPI clock
        mspi_fast_hs_clk: Some(MspiFastHsClkConfig::_5),
        mspi_fast_ls_clk: None, // Unused when root clock is PLL
        apb_clk: Some(ApbClkConfig::new(0)),
        ledc_sclk: Some(LedcSclkConfig::PllF80m),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
    };
    const PRESET_160: ClockConfig = ClockConfig {
        xtal_clk: None,
        soc_root_clk: Some(SocRootClkConfig::Pll),
        cpu_hs_div: Some(CpuHsDivConfig::_0),
        cpu_ls_div: None, // Unused when root clock is PLL
        ahb_hs_div: Some(AhbHsDivConfig::_3),
        ahb_ls_div: None, // Unused when root clock is PLL
        // Configures 80MHz MSPI clock
        mspi_fast_hs_clk: Some(MspiFastHsClkConfig::_5),
        mspi_fast_ls_clk: None, // Unused when root clock is PLL
        apb_clk: Some(ApbClkConfig::new(0)),
        ledc_sclk: Some(LedcSclkConfig::PllF80m),
        lp_fast_clk: Some(LpFastClkConfig::RcFastClk),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
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

        // On ESP32C6, MSPI source clock's default HS divider leads to 120MHz, which is unusable
        // before calibration. Therefore, before switching SOC_ROOT_CLK to HS, we need to set
        // MSPI source clock HS divider to make it run at 80MHz after the switch.
        // PLL = 480MHz, so divider is 6.
        ClockTree::with(|clocks| configure_mspi_fast_hs_clk(clocks, MspiFastHsClkConfig::_5));

        self.apply();
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, _config: XtalClkConfig) {
    // The stored configuration affects PLL settings instead.
}

// PLL_CLK

fn enable_pll_clk_impl(_clocks: &mut ClockTree, en: bool) {
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
            w.tie_low_xpd_bb_i2c().set_bit();
            w.tie_low_xpd_bbpll().set_bit();
            w.tie_low_xpd_bbpll_i2c().set_bit()
        });

        return;
    }

    // enable i2c mst clk by temporarily forcing on
    let old_clk_conf = MODEM_LPCON::regs().clk_conf().read();
    MODEM_LPCON::regs().clk_conf().write(|w| {
        unsafe { w.bits(old_clk_conf.bits()) };
        w.clk_i2c_mst_en().set_bit()
    });

    MODEM_LPCON::regs()
        .i2c_mst_clk_conf()
        .modify(|_, w| w.clk_i2c_mst_sel_160m().set_bit());

    // BBPLL CALIBRATION START
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    const DIV_REF: u8 = 0; // Do not divide reference clock
    const DCHGP: u8 = 5;
    const DCUR: u8 = 3;

    const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
    const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;
    const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

    const I2C_BBPLL_LREF: u8 = (DCHGP << I2C_BBPLL_OC_DCHGP_LSB) | DIV_REF;
    const I2C_BBPLL_DCUR: u8 =
        (1 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (3 << I2C_BBPLL_OC_DHREF_SEL_LSB) | DCUR;

    regi2c::I2C_BBPLL_OC_REF.write_reg(I2C_BBPLL_LREF);
    regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(8); // multiply 40MHz XTAL by 8+4
    regi2c::I2C_BBPLL_OC_DR1.write_field(0);
    regi2c::I2C_BBPLL_OC_DR3.write_field(0);
    regi2c::I2C_BBPLL_REG6.write_reg(I2C_BBPLL_DCUR);
    regi2c::I2C_BBPLL_OC_VCO_DBIAS.write_field(2);

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

    MODEM_LPCON::regs()
        .clk_conf()
        .write(|w| unsafe { w.bits(old_clk_conf.bits()) });
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_fosc_clk().bit(en));
    // TODO: Should the digital clock gate be a different clock node?
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_fosc().bit(en));
    crate::rom::ets_delay_us(5);
}

// XTAL32K_CLK

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LP_CLKRST::regs().xtal32k().write(|w| unsafe {
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
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_xtal32k().bit(en));
}

// OSC_SLOW_CLK

fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // TODO:
    // gpio_ll_input_enable(&GPIO, SOC_EXT_OSC_SLOW_GPIO_NUM);
    // REG_SET_BIT(LP_AON_GPIO_HOLD0_REG, BIT(SOC_EXT_OSC_SLOW_GPIO_NUM));
    todo!();

    // No need to configure anything else for OSC_SLOW_CLK
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        // SCK_DCAP value controls tuning of 136k clock. The higher the value of DCAP, the lower the
        // frequency. There is no separate enable bit, just make sure the calibration value is set.
        const RTC_CNTL_SCK_DCAP_DEFAULT: u8 = 128;
        crate::soc::regi2c::I2C_DIG_REG_SCK_DCAP.write_reg(RTC_CNTL_SCK_DCAP_DEFAULT);
    }

    PMU::regs()
        .hp_sleep_lp_ck_power()
        .modify(|_, w| w.hp_sleep_xpd_rc32k().bit(en));

    // Enable for digital part
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_osc32k().bit(en));
}

// HP_ROOT_CLK

fn enable_hp_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, managed by hardware.
}

fn configure_hp_root_clk_impl(_clocks: &mut ClockTree, _new_config: HpRootClkConfig) {
    // Nothing to do, managed by hardware.
}

// CPU_CLK

fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<CpuClkConfig>,
    _new_selector: CpuClkConfig,
) {
    // Nothing to do, automatically managed by hardware.
}

// AHB_CLK

fn configure_ahb_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<AhbClkConfig>,
    _new_selector: AhbClkConfig,
) {
    // Nothing to do, automatically managed by hardware.
}

// MSPI_FAST_CLK

fn enable_mspi_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_mspi_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<MspiFastClkConfig>,
    _new_selector: MspiFastClkConfig,
) {
    // Nothing to do, automatically managed by hardware.
}

// SOC_ROOT_CLK

fn enable_soc_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_soc_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<SocRootClkConfig>,
    new_selector: SocRootClkConfig,
) {
    PCR::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_selector {
            SocRootClkConfig::Xtal => 0,
            SocRootClkConfig::Pll => 1,
            SocRootClkConfig::RcFast => 2,
        })
    });
}

// CPU_HS_DIV

fn enable_cpu_hs_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_hs_div_impl(_clocks: &mut ClockTree, new_config: CpuHsDivConfig) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_hs_div_num().bits(new_config.value() as u8) });
}

// CPU_LS_DIV

fn enable_cpu_ls_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_ls_div_impl(_clocks: &mut ClockTree, new_config: CpuLsDivConfig) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_ls_div_num().bits(new_config.value() as u8) });
}

// AHB_HS_DIV

fn enable_ahb_hs_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ahb_hs_div_impl(_clocks: &mut ClockTree, new_config: AhbHsDivConfig) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_hs_div_num().bits(new_config.value() as u8) });
}

// AHB_LS_DIV

fn enable_ahb_ls_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ahb_ls_div_impl(_clocks: &mut ClockTree, new_config: AhbLsDivConfig) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_ls_div_num().bits(new_config.value() as u8) });
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

// MSPI_FAST_HS_CLK

fn enable_mspi_fast_hs_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, automatically managed by hardware.
}

fn configure_mspi_fast_hs_clk_impl(_clocks: &mut ClockTree, new_config: MspiFastHsClkConfig) {
    PCR::regs()
        .mspi_clk_conf()
        .modify(|_, w| unsafe { w.mspi_fast_hs_div_num().bits(new_config.value() as u8) });
}

// MSPI_FAST_LS_CLK

fn enable_mspi_fast_ls_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, automatically managed by hardware.
}

fn configure_mspi_fast_ls_clk_impl(_clocks: &mut ClockTree, new_config: MspiFastLsClkConfig) {
    PCR::regs()
        .mspi_clk_conf()
        .modify(|_, w| unsafe { w.mspi_fast_ls_div_num().bits(new_config.value() as u8) });
}

// PLL_F48M

fn enable_pll_f48m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_F80M

fn enable_pll_f80m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_F160M

fn enable_pll_f160m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_F240M

fn enable_pll_f240m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// LEDC_SCLK

fn enable_ledc_sclk_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .ledc_sclk_conf()
        .modify(|_, w| w.ledc_sclk_en().bit(en));
}

fn configure_ledc_sclk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LedcSclkConfig>,
    new_selector: LedcSclkConfig,
) {
    PCR::regs().ledc_sclk_conf().modify(|_, w| unsafe {
        w.ledc_sclk_sel().bits(match new_selector {
            LedcSclkConfig::PllF80m => 1,
            LedcSclkConfig::RcFastClk => 2,
            LedcSclkConfig::XtalClk => 3,
        })
    });
}

// XTAL_D2_CLK

fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, the divider is always on.
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
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| {
        w.fast_clk_sel().bit(match new_selector {
            LpFastClkConfig::RcFastClk => false,
            LpFastClkConfig::XtalD2Clk => true,
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
            LpSlowClkConfig::Xtal32kClk => 1,
            LpSlowClkConfig::RcSlow => 0,
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
            Mcpwm0FunctionClockConfig::PllF160m => 1,
            Mcpwm0FunctionClockConfig::XtalClk => 2,
            Mcpwm0FunctionClockConfig::RcFastClk => 3,
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
                Timg0FunctionClockConfig::RcFastClk => 2,
                Timg0FunctionClockConfig::PllF80m => 1,
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
                Timg0WdtClockConfig::PllF80m => 1,
                Timg0WdtClockConfig::RcFastClk => 2,
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
                Timg0FunctionClockConfig::RcFastClk => 2,
                Timg0FunctionClockConfig::PllF80m => 1,
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
                Timg0WdtClockConfig::PllF80m => 1,
                Timg0WdtClockConfig::RcFastClk => 2,
            })
        });
}

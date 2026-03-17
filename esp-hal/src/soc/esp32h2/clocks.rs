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
#![allow(missing_docs, reason = "Experimental")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use crate::{
    peripherals::{I2C_ANA_MST, LP_CLKRST, MODEM_LPCON, PCR, PMU, TIMG0, TIMG1, UART0, UART1},
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

fn configure_xtal_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<XtalClkConfig>,
    _config: XtalClkConfig,
) {
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
    _old_config: Option<HpRootClkConfig>,
    new_config: HpRootClkConfig,
) {
    PCR::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_config {
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

fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuClkConfig>,
    new_config: CpuClkConfig,
) {
    PCR::regs()
        .cpu_freq_conf()
        .modify(|_, w| unsafe { w.cpu_div_num().bits(new_config.divisor() as u8) });

    // "When selecting the clock source of HP_ROOT_CLK, or configuring the clock divisor for CPU_CLK
    // and AHB_CLK ..."
    clk_ll_bus_update();
}

// AHB_CLK

fn enable_ahb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do
}

fn configure_ahb_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<AhbClkConfig>,
    new_config: AhbClkConfig,
) {
    PCR::regs()
        .ahb_freq_conf()
        .modify(|_, w| unsafe { w.ahb_div_num().bits(new_config.divisor() as u8) });

    // "When selecting the clock source of HP_ROOT_CLK, or configuring the clock divisor for CPU_CLK
    // and AHB_CLK ..."
    clk_ll_bus_update();
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
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
    // Nothing to do.
}

// LP_FAST_CLK

fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
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
    _old_config: Option<LpSlowClkConfig>,
    new_config: LpSlowClkConfig,
) {
    LP_CLKRST::regs().lp_clk_conf().modify(|_, w| unsafe {
        w.slow_clk_sel().bits(match new_config {
            LpSlowClkConfig::RcSlow => 0,
            LpSlowClkConfig::Xtal32k => 1,
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
    _old_config: Option<McpwmFunctionClockConfig>,
    new_config: McpwmFunctionClockConfig,
) {
    PCR::regs().pwm_clk_conf().modify(|_, w| unsafe {
        w.pwm_clkm_sel().bits(match new_config {
            McpwmFunctionClockConfig::XtalClk => 0,
            McpwmFunctionClockConfig::RcFastClk => 1,
            McpwmFunctionClockConfig::PllF96m => 2,
        })
    });
}

// TIMG0_FUNCTION_CLOCK

fn enable_timg0_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup(0)
        .timer_clk_conf()
        .modify(|_, w| w.timer_clk_en().bit(en));
}

fn configure_timg0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgFunctionClockConfig>,
    new_config: TimgFunctionClockConfig,
) {
    // TODO: add variants to PAC
    PCR::regs()
        .timergroup(0)
        .timer_clk_conf()
        .modify(|_, w| unsafe {
            w.timer_clk_sel().bits(match new_config {
                TimgFunctionClockConfig::XtalClk => 0,
                TimgFunctionClockConfig::RcFastClk => 1,
                TimgFunctionClockConfig::PllF48m => 2,
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
    _old_config: Option<TimgCalibrationClockConfig>,
    new_config: TimgCalibrationClockConfig,
) {
    TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_clk_sel().bits(match new_config {
            TimgCalibrationClockConfig::RcSlowClk => 0,
            TimgCalibrationClockConfig::RcFastDivClk => 1,
            TimgCalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

// TIMG0_WDT_CLOCK

fn enable_timg0_wdt_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup(0)
        .wdt_clk_conf()
        .modify(|_, w| w.wdt_clk_en().bit(en));
}

fn configure_timg0_wdt_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgWdtClockConfig>,
    new_config: TimgWdtClockConfig,
) {
    PCR::regs()
        .timergroup(0)
        .wdt_clk_conf()
        .modify(|_, w| unsafe {
            w.wdt_clk_sel().bits(match new_config {
                TimgWdtClockConfig::XtalClk => 0,
                TimgWdtClockConfig::RcFastClk => 1,
                TimgWdtClockConfig::PllF48m => 2,
            })
        });
}

// PARL_IO_RX_CLOCK

fn enable_parl_io_rx_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .parl_clk_rx_conf()
        .modify(|_, w| w.parl_clk_rx_en().bit(en));
}

fn configure_parl_io_rx_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<ParlIoRxClockConfig>,
    new_config: ParlIoRxClockConfig,
) {
    PCR::regs().parl_clk_rx_conf().modify(|_, w| unsafe {
        w.parl_clk_rx_sel().bits(match new_config {
            ParlIoRxClockConfig::XtalClk => 0,
            ParlIoRxClockConfig::RcFastClk => 2,
            ParlIoRxClockConfig::PllF96m => 1,
        })
    });
}

// PARL_IO_TX_CLOCK

fn enable_parl_io_tx_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .parl_clk_tx_conf()
        .modify(|_, w| w.parl_clk_tx_en().bit(en));
}

fn configure_parl_io_tx_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<ParlIoTxClockConfig>,
    new_config: ParlIoTxClockConfig,
) {
    PCR::regs().parl_clk_tx_conf().modify(|_, w| unsafe {
        w.parl_clk_tx_sel().bits(match new_config {
            ParlIoTxClockConfig::XtalClk => 0,
            ParlIoTxClockConfig::RcFastClk => 2,
            ParlIoTxClockConfig::PllF96m => 1,
        })
    });
}

// RMT_SCLK

fn enable_rmt_sclk_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .rmt_sclk_conf()
        .modify(|_, w| w.sclk_en().bit(en));
}

fn configure_rmt_sclk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<RmtSclkConfig>,
    new_config: RmtSclkConfig,
) {
    PCR::regs().rmt_sclk_conf().modify(|_, w| {
        w.sclk_sel().bit(match new_config {
            RmtSclkConfig::XtalClk => false,
            RmtSclkConfig::RcFastClk => true,
        })
    });
}

// TIMG1_FUNCTION_CLOCK

fn enable_timg1_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup(1)
        .timer_clk_conf()
        .modify(|_, w| w.timer_clk_en().bit(en));
}

fn configure_timg1_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgFunctionClockConfig>,
    new_config: TimgFunctionClockConfig,
) {
    // TODO: add variants to PAC
    PCR::regs()
        .timergroup(1)
        .timer_clk_conf()
        .modify(|_, w| unsafe {
            w.timer_clk_sel().bits(match new_config {
                TimgFunctionClockConfig::XtalClk => 0,
                TimgFunctionClockConfig::RcFastClk => 1,
                TimgFunctionClockConfig::PllF48m => 2,
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
    _old_config: Option<TimgCalibrationClockConfig>,
    new_config: TimgCalibrationClockConfig,
) {
    TIMG1::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_clk_sel().bits(match new_config {
            TimgCalibrationClockConfig::RcSlowClk => 0,
            TimgCalibrationClockConfig::RcFastDivClk => 1,
            TimgCalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

// TIMG1_WDT_CLOCK

fn enable_timg1_wdt_clock_impl(_clocks: &mut ClockTree, en: bool) {
    PCR::regs()
        .timergroup(1)
        .wdt_clk_conf()
        .modify(|_, w| w.wdt_clk_en().bit(en));
}

fn configure_timg1_wdt_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgWdtClockConfig>,
    new_config: TimgWdtClockConfig,
) {
    PCR::regs()
        .timergroup(1)
        .wdt_clk_conf()
        .modify(|_, w| unsafe {
            w.wdt_clk_sel().bits(match new_config {
                TimgWdtClockConfig::XtalClk => 0,
                TimgWdtClockConfig::RcFastClk => 1,
                TimgWdtClockConfig::PllF48m => 2,
            })
        });
}

// UART0_FUNCTION_CLOCK

fn enable_uart0_function_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // At least on revision 0.1 switching SCLK off causes the chip to no longer boot.
    // TODO: https://github.com/esp-rs/esp-hal/issues/4952
    // enable_uart_function_clock(0, en);
}

fn configure_uart0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<UartFunctionClockConfig>,
    new_config: UartFunctionClockConfig,
) {
    configure_uart_function_clock(0, new_config);
}

// UART0_BAUD_RATE_GENERATOR

fn enable_uart0_baud_rate_generator_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do
}

fn configure_uart0_baud_rate_generator_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<UartBaudRateGeneratorConfig>,
    new_config: UartBaudRateGeneratorConfig,
) {
    UART0::regs().clkdiv().write(|w| unsafe {
        w.clkdiv().bits(new_config.integral as _);
        w.frag().bits(new_config.fractional as _)
    });
}

// UART1_FUNCTION_CLOCK

fn enable_uart1_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    enable_uart_function_clock(1, en);
}

fn configure_uart1_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<UartFunctionClockConfig>,
    new_config: UartFunctionClockConfig,
) {
    configure_uart_function_clock(1, new_config);
}

// UART1_BAUD_RATE_GENERATOR

fn enable_uart1_baud_rate_generator_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do
}

fn configure_uart1_baud_rate_generator_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<UartBaudRateGeneratorConfig>,
    new_config: UartBaudRateGeneratorConfig,
) {
    UART1::regs().clkdiv().write(|w| unsafe {
        w.clkdiv().bits(new_config.integral as _);
        w.frag().bits(new_config.fractional as _)
    });
}

fn enable_uart_function_clock(uart: usize, en: bool) {
    PCR::regs()
        .uart(uart)
        .clk_conf()
        .modify(|_, w| w.sclk_en().bit(en));
}

fn configure_uart_function_clock(uart: usize, new_config: UartFunctionClockConfig) {
    PCR::regs().uart(uart).clk_conf().modify(|_, w| unsafe {
        w.sclk_sel().bits(match new_config.sclk {
            UartFunctionClockSclk::PllF48m => 1,
            UartFunctionClockSclk::RcFast => 2,
            UartFunctionClockSclk::Xtal => 3,
        });
        w.sclk_div_a().bits(0);
        w.sclk_div_b().bits(0);
        w.sclk_div_num().bits(new_config.div_num as _);
        w
    });
}

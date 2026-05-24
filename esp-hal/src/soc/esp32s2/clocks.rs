//! Clock tree definitions and implementations for ESP32-S2.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (90k RC_SLOW and 8M RC_FAST) are not calibrated here, this system can
//!   only give a rough estimate of their frequency. They can be calibrated separately using a known
//!   crystal frequency.
//! - Some of the SOC capabilities are not implemented: using external 32K oscillator instead of a
//!   crystal, divider for CLK8M and possibly more.
#![allow(dead_code, reason = "Some of this is bound to be unused")]
#![allow(missing_docs, reason = "Experimental")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use esp_rom_sys::rom::{ets_delay_us, ets_update_cpu_frequency_rom};

use crate::{
    peripherals::{I2C_ANA_MST, LPWR, RMT, SYSCON, SYSTEM, TIMG0, TIMG1, UART0, UART1},
    soc::regi2c,
    time::Rate,
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
    // Crypto peripherals don't like 320MHz PLL.
    const PRESET_80: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_480),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::new(CpuPllDivDivisor::_6)),
        system_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
        timg_calibration_clock: None,
    };
    const PRESET_160: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_480),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::new(CpuPllDivDivisor::_3)),
        system_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
        timg_calibration_clock: None,
    };
    const PRESET_240: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_480),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::new(CpuPllDivDivisor::_2)),
        system_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
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

    pub(crate) fn configure(mut self, clocks: &mut ClockTree) {
        if self.xtal_clk.is_none() {
            self.xtal_clk = Some(XtalClkConfig::_40);
        }

        // Switch CPU to XTAL before reconfiguring PLL.
        configure_xtal_clk(clocks, XtalClkConfig::_40);
        configure_system_pre_div(clocks, SystemPreDivConfig::new(0));
        configure_cpu_clk(clocks, CpuClkConfig::Xtal);

        self.apply(clocks);
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<XtalClkConfig>,
    _config: XtalClkConfig,
) {
    // The stored configuration affects PLL settings instead.
}

// PLL_CLK

fn enable_pll_clk_impl(clocks: &mut ClockTree, en: bool) {
    let power_down = !en;

    // regi2c_ctrl_ll_i2c_bbpll_enable
    // regi2c_ctrl_ll_i2c_apll_enable
    I2C_ANA_MST::regs().config1().modify(|_, w| {
        w.bbpll().bit(power_down);
        w.apll().bit(power_down)
    });

    LPWR::regs().options0().modify(|_, w| {
        w.bb_i2c_force_pd().bit(power_down);
        w.bbpll_force_pd().bit(power_down);
        w.bbpll_i2c_force_pd().bit(power_down)
    });

    if !en {
        return;
    }

    ensure_voltage_raised(clocks);

    // Analog part
    let div_ref: u8;
    let div7_0: u8;
    let dr1: u8;
    let dr3: u8;
    let dchgp: u8;
    let dcur: u8;
    let mode_hf: u8;

    match unwrap!(clocks.pll_clk) {
        PllClkConfig::_480 => {
            div_ref = 0;
            div7_0 = 8;
            dr1 = 0;
            dr3 = 0;
            dchgp = 5;
            dcur = 4;
            mode_hf = 0x6B;
        }
        PllClkConfig::_320 => {
            div_ref = 0;
            div7_0 = 4;
            dr1 = 0;
            dr3 = 0;
            dchgp = 5;
            dcur = 5;
            mode_hf = 0x69;
        }
    }

    const I2C_BBPLL_OC_DCHGP_LSB: u8 = 4;
    const I2C_BBPLL_OC_DLREF_SEL_LSB: u8 = 6;
    const I2C_BBPLL_OC_DHREF_SEL_LSB: u8 = 4;

    regi2c::I2C_BBPLL_REG4.write_reg(mode_hf);
    let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | (div_ref);
    let i2c_bbpll_dcur =
        (2 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (1 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;
    regi2c::I2C_BBPLL_REG2.write_reg(i2c_bbpll_lref);
    regi2c::I2C_BBPLL_REG3.write_reg(div7_0);

    regi2c::I2C_BBPLL_OC_DR1.write_field(dr1);
    regi2c::I2C_BBPLL_OC_DR3.write_field(dr3);
    regi2c::I2C_BBPLL_REG6.write_reg(i2c_bbpll_dcur);

    regi2c::I2C_BBPLL_IR_CAL_ENX_CAP.write_field(1);

    let mut success = false;
    for ext_cap in 0..16 {
        regi2c::I2C_BBPLL_IR_CAL_EXT_CAP.write_field(ext_cap);

        if regi2c::I2C_BBPLL_OR_CAL_CAP.read() == 0 {
            success |= true;
            break;
        }
    }

    assert!(success, "BBPLL calibration failed");

    ensure_voltage_minimal(clocks);
}

fn configure_pll_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<PllClkConfig>,
    _config: PllClkConfig,
) {
    // Nothing to do. The PLL may still be powered down. We'll configure it in
    // `enable_pll_clk_impl`.
}

// APLL_CLK

fn enable_apll_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LPWR::regs().ana_conf().modify(|_, w| {
        w.plla_force_pd().bit(!en);
        w.plla_force_pu().bit(en)
    });
    if en {
        todo!(
            "Implement APLL configuration and calibration here. See esp-idf `clk_ll_apll_set_config`"
        );
    }
}

fn configure_apll_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<ApllClkConfig>,
    _config: ApllClkConfig,
) {
    // Nothing to do. The APLL may still be powered down. We'll configure it in
    // `enable_apll_clk_impl`.
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    const CLK_LL_RC_FAST_ENABLE_WAIT_DEFAULT: u8 = 5;
    const CLK_LL_RC_FAST_WAIT_DEFAULT: u8 = 20;
    LPWR::regs().clk_conf().modify(|_, w| w.enb_ck8m().bit(!en));
    LPWR::regs().timer1().modify(|_, w| unsafe {
        w.ck8m_wait().bits(if en {
            CLK_LL_RC_FAST_ENABLE_WAIT_DEFAULT
        } else {
            CLK_LL_RC_FAST_WAIT_DEFAULT
        })
    });
    if en {
        ets_delay_us(50);
    }
}

// CPU_PLL_DIV_IN

fn enable_cpu_pll_div_in_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_pll_div_in_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuPllDivInConfig>,
    _new_config: CpuPllDivInConfig,
) {
    // Nothing to do.
}

// CPU_PLL_DIV

fn enable_cpu_pll_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_pll_div_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuPllDivConfig>,
    _new_config: CpuPllDivConfig,
) {
    // Nothing to do.
}

// SYSTEM_PRE_DIV_IN

fn enable_system_pre_div_in_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_system_pre_div_in_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<SystemPreDivInConfig>,
    _new_config: SystemPreDivInConfig,
) {
    // Nothing to do.
}

// SYSTEM_PRE_DIV

fn enable_system_pre_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_system_pre_div_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<SystemPreDivConfig>,
    new_config: SystemPreDivConfig,
) {
    SYSTEM::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.pre_div_cnt().bits(new_config.divisor() as u16 & 0x3FF) });
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_apb_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<ApbClkConfig>,
    _new_config: ApbClkConfig,
) {
    // Nothing to do.
}

// REF_TICK

fn enable_ref_tick_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<RefTickConfig>,
    _new_config: RefTickConfig,
) {
    // Nothing to do.
}

// REF_TICK_XTAL

fn enable_ref_tick_xtal_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_xtal_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<RefTickXtalConfig>,
    new_config: RefTickXtalConfig,
) {
    SYSCON::regs()
        .tick_conf()
        .modify(|_, w| unsafe { w.xtal_tick_num().bits(new_config.divisor() as u8) });
}

// REF_TICK_CK8M

fn enable_ref_tick_ck8m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_ck8m_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<RefTickCk8mConfig>,
    new_config: RefTickCk8mConfig,
) {
    SYSCON::regs()
        .tick_conf()
        .modify(|_, w| unsafe { w.ck8m_tick_num().bits(new_config.divisor() as u8) });
}

// CPU_CLK

fn configure_cpu_clk_impl(
    clocks: &mut ClockTree,
    _old_config: Option<CpuClkConfig>,
    new_config: CpuClkConfig,
) {
    // Based on TRM Table 6.2-2
    let clock_source_sel0_bit = match new_config {
        CpuClkConfig::Xtal => 0,
        CpuClkConfig::RcFast => 2,
        CpuClkConfig::Apll => 3,
        CpuClkConfig::Pll => 1,
    };
    let clock_source_sel1_bit = clocks.pll_clk == Some(PllClkConfig::_480);
    let clock_source_sel2_bit = match (clocks.pll_clk, clocks.cpu_pll_div) {
        (Some(PllClkConfig::_480), Some(div)) if div.divisor() == 6 => 0,
        (Some(PllClkConfig::_480), Some(div)) if div.divisor() == 3 => 1,
        (Some(PllClkConfig::_480), Some(div)) if div.divisor() == 2 => 2,

        // 320 MHz or APLL
        (_, Some(div)) if div.divisor() == 4 => 0,
        (_, Some(div)) if div.divisor() == 2 => 1,

        // don't care
        _ => 0,
    };

    ensure_voltage_raised(clocks);

    if new_config == CpuClkConfig::Pll {
        SYSTEM::regs().cpu_per_conf().modify(|_, w| {
            unsafe { w.cpuperiod_sel().bits(clock_source_sel2_bit) };
            w.pll_freq_sel().bit(clock_source_sel1_bit)
        });
    }

    SYSTEM::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.soc_clk_sel().bits(clock_source_sel0_bit) });

    // Store frequencies in expected places.
    let cpu_freq = Rate::from_hz(cpu_clk_frequency());
    ets_update_cpu_frequency_rom(cpu_freq.as_mhz());

    let apb_freq = Rate::from_hz(apb_clk_frequency());
    update_apb_frequency(apb_freq);

    ensure_voltage_minimal(clocks);
}

fn update_apb_frequency(freq: Rate) {
    let freq_shifted = (freq.as_hz() >> 12) & 0xFFFF;
    let value = freq_shifted | (freq_shifted << 16);
    LPWR::regs()
        .store5()
        .modify(|_, w| unsafe { w.data().bits(value) });
}

fn uses_80mhz_flash() -> bool {
    unsafe {
        crate::soc::pac::SPI0::steal()
            .clock()
            .read()
            .clk_equ_sysclk()
            .bit_is_set()
    }
}
fn is_max_cpu_speed(clocks: &mut ClockTree) -> bool {
    clocks.cpu_clk == Some(CpuClkConfig::Pll)
        && clocks.pll_clk == Some(PllClkConfig::_480)
        && matches!(clocks.cpu_pll_div, Some(div) if div.divisor() == 2)
}

const RTC_CNTL_DBIAS_1V10: u8 = 4;
const RTC_CNTL_DBIAS_1V25: u8 = 7;

fn ensure_voltage_raised(clocks: &mut ClockTree) {
    if is_max_cpu_speed(clocks) || uses_80mhz_flash() {
        LPWR::regs().reg().modify(|_, w| unsafe {
            w.dig_reg_dbias_wak().bits(RTC_CNTL_DBIAS_1V25);
            w.dbias_wak().bits(RTC_CNTL_DBIAS_1V25)
        });

        ets_delay_us(40);
    }
}

fn ensure_voltage_minimal(clocks: &mut ClockTree) {
    if !is_max_cpu_speed(clocks) {
        LPWR::regs().reg().modify(|_, w| unsafe {
            if uses_80mhz_flash() {
                w.dig_reg_dbias_wak().bits(RTC_CNTL_DBIAS_1V25);
            } else {
                w.dig_reg_dbias_wak().bits(RTC_CNTL_DBIAS_1V10);
            }
            w.dbias_wak().bits(RTC_CNTL_DBIAS_1V10)
        });
        ets_delay_us(40);
    }
}

// APB_CLK_CPU_DIV2

fn enable_apb_clk_cpu_div2_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// APB_CLK_80M

fn enable_apb_clk_80m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// XTAL32K_CLK

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    // This bit only enables the clock for the digital core, what about RTC? Should we split the
    // clock node in two?
    LPWR::regs().ext_xtl_conf().modify(|_, w| unsafe {
        w.dac_xtal_32k().bits(3);
        w.dres_xtal_32k().bits(3);
        w.dgm_xtal_32k().bits(3);
        w.dbuf_xtal_32k().bit(true);
        w.xpd_xtal_32k().bit(en);
        w.xtal32k_xpd_force().bit(!en)
    });

    // TODO: external oscillator may need different settings

    // Enable for digital part
    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.dig_xtal32k_en().bit(en));
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        // SCK_DCAP value controls tuning of the 90k clock. The higher the value of DCAP, the lower
        // the frequency. There is no separate enable bit, just make sure the calibration
        // value is set.
        const RTC_CNTL_SCK_DCAP_DEFAULT: u8 = 255;
        LPWR::regs()
            .reg()
            .modify(|_, w| unsafe { w.sck_dcap().bits(RTC_CNTL_SCK_DCAP_DEFAULT) });

        // Also configure the divider here to its usual value of 1.

        // Updating the divider should be part of the RC_SLOW_CLK divider config:
        let slow_clk_conf = LPWR::regs().slow_clk_conf();
        // Invalidate
        let new_value = slow_clk_conf.modify(|_, w| w.ana_clk_div_vld().clear_bit());
        // Update divider
        let new_value = slow_clk_conf.write(|w| unsafe {
            w.bits(new_value);
            w.ana_clk_div().bits(0)
        });
        // Re-synchronize
        slow_clk_conf.write(|w| {
            unsafe { w.bits(new_value) };
            w.ana_clk_div_vld().set_bit()
        });
    }
}

// RC_FAST_DIV_CLK

fn enable_rc_fast_div_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LPWR::regs().clk_conf().modify(|_, w| {
        // Active-low: clear to enable, set to disable.
        w.enb_ck8m_div().bit(!en);
        unsafe { w.ck8m_div().bits(1) } // divide by 256
    });
}

// XTAL_DIV_CLK

fn enable_xtal_div_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// RTC_SLOW_CLK

fn enable_rtc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_rtc_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<RtcSlowClkConfig>,
    new_config: RtcSlowClkConfig,
) {
    LPWR::regs().clk_conf().modify(|_, w| unsafe {
        w.ana_clk_rtc_sel().bits(match new_config {
            RtcSlowClkConfig::RcSlow => 0,
            RtcSlowClkConfig::Xtal32k => 1,
            RtcSlowClkConfig::RcFast => 2,
        })
    });

    ets_delay_us(300);
}

// RTC_FAST_CLK

fn enable_rtc_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_rtc_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<RtcFastClkConfig>,
    new_config: RtcFastClkConfig,
) {
    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.fast_clk_rtc_sel().bit(new_config == RtcFastClkConfig::Rc));

    ets_delay_us(3);
}

// UART_MEM_CLK

fn enable_uart_mem_clk_impl(_clocks: &mut ClockTree, en: bool) {
    // TODO: these functions (peripheral bus clock control) should be generated,
    // replacing current PeripheralClockControl code.
    // Enabling clock should probably not reset the peripheral.
    let regs = SYSTEM::regs();

    if en {
        regs.perip_rst_en0()
            .modify(|_, w| w.uart_mem_rst().bit(true));
        regs.perip_rst_en0()
            .modify(|_, w| w.uart_mem_rst().bit(false));
    }

    regs.perip_clk_en0()
        .modify(|_, w| w.uart_mem_clk_en().bit(en));
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
    TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_clk_sel().bits(match new_config {
            TimgCalibrationClockConfig::RtcClk => 0,
            TimgCalibrationClockConfig::RcFastDivClk => 1,
            TimgCalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

impl TimgInstance {
    // TIMG_FUNCTION_CLOCK

    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, en: bool) {
        let regs = match self {
            TimgInstance::Timg0 => TIMG0::regs(),
            TimgInstance::Timg1 => TIMG1::regs(),
        };
        regs.regclk().modify(|_, w| w.clk_en().bit(en));
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgFunctionClockConfig>,
        new_config: TimgFunctionClockConfig,
    ) {
        let regs = match self {
            TimgInstance::Timg0 => TIMG0::regs(),
            TimgInstance::Timg1 => TIMG1::regs(),
        };
        regs.t(0).config().modify(|_, w| {
            w.use_xtal()
                .bit(new_config == TimgFunctionClockConfig::XtalClk)
        });
        regs.t(1).config().modify(|_, w| {
            w.use_xtal()
                .bit(new_config == TimgFunctionClockConfig::XtalClk)
        });
    }
}

impl RmtInstance {
    // RMT_SCLK

    fn enable_sclk_impl(self, _clocks: &mut ClockTree, en: bool) {
        RMT::regs().apb_conf().modify(|_, w| w.clk_en().bit(en));
    }

    fn configure_sclk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<RmtSclkConfig>,
        new_config: RmtSclkConfig,
    ) {
        for ch_num in 0..4 {
            RMT::regs()
                .chconf1(ch_num)
                .modify(|_, w| w.ref_always_on().bit(new_config == RmtSclkConfig::ApbClk));
        }
    }
}

impl UartInstance {
    // UART_FUNCTION_CLOCK

    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        new_config: UartFunctionClockConfig,
    ) {
        let regs = match self {
            UartInstance::Uart0 => UART0::regs(),
            UartInstance::Uart1 => UART1::regs(),
        };
        regs.conf0().modify(|_, w| {
            w.tick_ref_always_on()
                .bit(new_config.sclk == UartFunctionClockSclk::Apb)
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

    // UART_MEM_CLOCK

    fn enable_mem_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Nothing to do.
    }

    fn configure_mem_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartMemClockConfig>,
        _new_config: UartMemClockConfig,
    ) {
        // Nothing to do.
    }
}

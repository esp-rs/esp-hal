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

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use esp_rom_sys::rom::{ets_delay_us, ets_update_cpu_frequency_rom};

use crate::{
    peripherals::{LPWR, SYSCON, SYSTEM, TIMG0, TIMG1},
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
        cpu_pll_div: Some(CpuPllDivConfig::_6),
        system_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
    };
    const PRESET_160: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_480),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::_3),
        system_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
    };
    const PRESET_240: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_480),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::_2),
        system_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
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
        if self.xtal_clk.is_none() {
            self.xtal_clk = Some(XtalClkConfig::_40);
        }

        self.apply();
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, _config: XtalClkConfig) {
    // The stored configuration affects PLL settings instead.
}

// PLL_CLK

fn enable_pll_clk_impl(clocks: &mut ClockTree, en: bool) {
    let power_down = !en;
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

fn configure_pll_clk_impl(_clocks: &mut ClockTree, _config: PllClkConfig) {
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

fn configure_apll_clk_impl(_clocks: &mut ClockTree, _config: ApllClkConfig) {
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
}

// CPU_PLL_DIV_IN

fn enable_cpu_pll_div_in_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_pll_div_in_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<CpuPllDivInConfig>,
    _new_selector: CpuPllDivInConfig,
) {
    // Nothing to do.
}

// CPU_PLL_DIV

fn enable_cpu_pll_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_pll_div_impl(_clocks: &mut ClockTree, _new_config: CpuPllDivConfig) {
    // Nothing to do.
}

// SYSTEM_PRE_DIV_IN

fn enable_system_pre_div_in_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_system_pre_div_in_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<SystemPreDivInConfig>,
    _new_selector: SystemPreDivInConfig,
) {
    // Nothing to do.
}

// SYSTEM_PRE_DIV

fn enable_system_pre_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_system_pre_div_impl(_clocks: &mut ClockTree, new_config: SystemPreDivConfig) {
    SYSTEM::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.pre_div_cnt().bits(new_config.value() as u16 & 0x3FF) });
}

// APB_CLK

fn enable_apb_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_apb_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<ApbClkConfig>,
    _new_selector: ApbClkConfig,
) {
    // Nothing to do.
}

// REF_TICK

fn enable_ref_tick_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<RefTickConfig>,
    _new_selector: RefTickConfig,
) {
    // Nothing to do.
}

// REF_TICK_XTAL

fn enable_ref_tick_xtal_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_xtal_impl(_clocks: &mut ClockTree, new_config: RefTickXtalConfig) {
    SYSCON::regs()
        .tick_conf()
        .modify(|_, w| unsafe { w.xtal_tick_num().bits(new_config.value() as u8) });
}

// REF_TICK_CK8M

fn enable_ref_tick_ck8m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_ck8m_impl(_clocks: &mut ClockTree, new_config: RefTickCk8mConfig) {
    SYSCON::regs()
        .tick_conf()
        .modify(|_, w| unsafe { w.ck8m_tick_num().bits(new_config.value() as u8) });
}

// CPU_CLK

fn configure_cpu_clk_impl(
    clocks: &mut ClockTree,
    _old_selector: Option<CpuClkConfig>,
    new_selector: CpuClkConfig,
) {
    // Based on TRM Table 6.2-2
    let clock_source_sel0_bit = match new_selector {
        CpuClkConfig::Xtal => 0,
        CpuClkConfig::RcFast => 2,
        CpuClkConfig::Apll => 3,
        CpuClkConfig::Pll => 1,
    };
    let clock_source_sel1_bit = clocks.pll_clk == Some(PllClkConfig::_480);
    let clock_source_sel2_bit = match (clocks.pll_clk, clocks.cpu_pll_div) {
        (Some(PllClkConfig::_480), Some(CpuPllDivConfig::_6)) => 0,
        (Some(PllClkConfig::_480), Some(CpuPllDivConfig::_3)) => 1,
        (Some(PllClkConfig::_480), Some(CpuPllDivConfig::_2)) => 2,

        // 320 MHz or APLL
        (_, Some(CpuPllDivConfig::_4)) => 0,
        (_, Some(CpuPllDivConfig::_2)) => 1,

        // don't care
        _ => 0,
    };

    ensure_voltage_raised(clocks);

    if new_selector == CpuClkConfig::Pll {
        SYSTEM::regs().cpu_per_conf().modify(|_, w| {
            unsafe { w.cpuperiod_sel().bits(clock_source_sel2_bit) };
            w.pll_freq_sel().bit(clock_source_sel1_bit)
        });
    }

    SYSTEM::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.soc_clk_sel().bits(clock_source_sel0_bit) });

    // Store frequencies in expected places.
    let cpu_freq = Rate::from_hz(cpu_clk_frequency(clocks));
    ets_update_cpu_frequency_rom(cpu_freq.as_mhz());

    let apb_freq = Rate::from_hz(apb_clk_frequency(clocks));
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

fn needs_high_voltage(clocks: &mut ClockTree) -> bool {
    let flash_frequency_80m = true; // TODO
    clocks.cpu_clk == Some(CpuClkConfig::Pll)
        && (clocks.pll_clk, clocks.cpu_pll_div)
            == (Some(PllClkConfig::_480), Some(CpuPllDivConfig::_2))
        || flash_frequency_80m
}

const RTC_CNTL_DBIAS_1V10: u8 = 4;
const RTC_CNTL_DBIAS_1V25: u8 = 7;

fn ensure_voltage_raised(clocks: &mut ClockTree) {
    if needs_high_voltage(clocks) {
        LPWR::regs().reg().modify(|_, w| unsafe {
            w.dig_reg_dbias_wak().bits(RTC_CNTL_DBIAS_1V25);
            w.dbias_wak().bits(RTC_CNTL_DBIAS_1V25)
        });

        ets_delay_us(40);
    }
}

fn ensure_voltage_minimal(clocks: &mut ClockTree) {
    if !needs_high_voltage(clocks) {
        LPWR::regs().reg().modify(|_, w| unsafe {
            w.dig_reg_dbias_wak().bits(RTC_CNTL_DBIAS_1V10);
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
        w.enb_ck8m_div().bit(en);
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
    _old_selector: Option<RtcSlowClkConfig>,
    new_selector: RtcSlowClkConfig,
) {
    LPWR::regs().clk_conf().modify(|_, w| unsafe {
        w.ana_clk_rtc_sel().bits(match new_selector {
            RtcSlowClkConfig::RcSlow => 0,
            RtcSlowClkConfig::Xtal => 1,
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
    _old_selector: Option<RtcFastClkConfig>,
    new_selector: RtcFastClkConfig,
) {
    LPWR::regs().clk_conf().modify(|_, w| {
        w.fast_clk_rtc_sel()
            .bit(new_selector == RtcFastClkConfig::Rc)
    });

    ets_delay_us(3);
}

// TIMG0_FUNCTION_CLOCK

// Note that the function clock is a pre-requisite of the timer, but does not enable the counter.

fn enable_timg0_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    TIMG0::regs().regclk().modify(|_, w| w.clk_en().bit(en));
}

fn configure_timg0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0FunctionClockConfig>,
    new_selector: Timg0FunctionClockConfig,
) {
    TIMG0::regs().t(0).config().modify(|_, w| {
        w.use_xtal()
            .bit(new_selector == Timg0FunctionClockConfig::XtalClk)
    });
    TIMG0::regs().t(1).config().modify(|_, w| {
        w.use_xtal()
            .bit(new_selector == Timg0FunctionClockConfig::XtalClk)
    });
}

// TIMG0_CALIBRATION_CLOCK

fn enable_timg0_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, calibration clocks can only be selected. They are gated by the CALI_START bit,
    // which is managed by the calibration process.
}

impl Timg0CalibrationClockConfig {
    fn cali_clk_sel_bits(self) -> u8 {
        match self {
            Timg0CalibrationClockConfig::RtcClk => 0,
            Timg0CalibrationClockConfig::RcFastDivClk => 1,
            Timg0CalibrationClockConfig::Xtal32kClk => 2,
        }
    }
}

fn configure_timg0_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0CalibrationClockConfig>,
    new_selector: Timg0CalibrationClockConfig,
) {
    TIMG0::regs()
        .rtccalicfg()
        .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(new_selector.cali_clk_sel_bits()) });
}

// TIMG1_FUNCTION_CLOCK

fn enable_timg1_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    TIMG1::regs().regclk().modify(|_, w| w.clk_en().bit(en));
}

fn configure_timg1_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0FunctionClockConfig>,
    new_selector: Timg0FunctionClockConfig,
) {
    TIMG1::regs().t(0).config().modify(|_, w| {
        w.use_xtal()
            .bit(new_selector == Timg0FunctionClockConfig::XtalClk)
    });
    TIMG1::regs().t(1).config().modify(|_, w| {
        w.use_xtal()
            .bit(new_selector == Timg0FunctionClockConfig::XtalClk)
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
    TIMG1::regs()
        .rtccalicfg()
        .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(new_selector.cali_clk_sel_bits()) });
}

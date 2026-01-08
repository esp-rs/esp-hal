//! Clock tree definitions and implementations for ESP32-C2.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (136k RC_SLOW and 17.5M RC_FAST) are not calibrated here, this system
//!   can only give a rough estimate of their frequency. They can be calibrated separately using a
//!   known crystal frequency.
//! - Some of the SOC capabilities are not implemented: using external 32K oscillator, divider for
//!   RC_FAST and possibly more.
#![allow(dead_code, reason = "Some of this is bound to be unused")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use esp_rom_sys::rom::{ets_delay_us, ets_update_cpu_frequency_rom};

use crate::{
    peripherals::{I2C_ANA_MST, LPWR, SYSTEM, TIMG0},
    rtc_cntl::Rtc,
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

    /// 120 MHz CPU clock
    _120MHz = 120,
}

impl CpuClock {
    const PRESET_80: ClockConfig = ClockConfig {
        xtal_clk: None,
        system_pre_div: None,
        cpu_pll_div: Some(CpuPllDivConfig::_6),
        cpu_clk: Some(CpuClkConfig::Pll),
        rc_fast_clk_div_n: Some(RcFastClkDivNConfig::new(0)),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
        low_power_clk: Some(LowPowerClkConfig::RtcSlow),
    };
    const PRESET_120: ClockConfig = ClockConfig {
        xtal_clk: None,
        system_pre_div: None,
        cpu_pll_div: Some(CpuPllDivConfig::_4),
        cpu_clk: Some(CpuClkConfig::Pll),
        rc_fast_clk_div_n: Some(RcFastClkDivNConfig::new(0)),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
        low_power_clk: Some(LowPowerClkConfig::RtcSlow),
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_80MHz => CpuClock::PRESET_80,
            CpuClock::_120MHz => CpuClock::PRESET_120,
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
            v if v == CpuClock::PRESET_120 => Some(CpuClock::_120MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(mut self) {
        // Detect XTAL if unset.
        // FIXME: this doesn't support running from RC_FAST_CLK. We should rework detection to only
        // run when requesting XTAL.
        ClockTree::with(|clocks| {
            if self.xtal_clk.is_none() {
                // While the bootloader stores a crystal frequency in a retention register,
                // that comes from a constant that we should not trust. If the user did not
                // provide a crystal frequency, we should detect it.
                let xtal = detect_xtal_freq(clocks);
                debug!("Auto-detected XTAL frequency: {}", xtal.value());
                self.xtal_clk = Some(xtal);
            }
        });

        self.apply();
    }
}

// TODO: this could be chip-independent
// We're rather impolite in this function by not saving and restoring configuration, but this is
// expected to run before configuring CPU clocks anyway.
fn detect_xtal_freq(clocks: &mut ClockTree) -> XtalClkConfig {
    const SLOW_CLOCK_CYCLES: u32 = 100;

    // Just an assumption for things to not panic.
    configure_xtal_clk(clocks, XtalClkConfig::_40);
    configure_system_pre_div(clocks, SystemPreDivConfig::new(0));
    configure_cpu_clk(clocks, CpuClkConfig::Xtal);

    // By default the TIMG0 bus clock is running. Do not create a peripheral guard as dropping it
    // would reset the timer, and it would enable its WDT.

    // Make sure the process doesn't time out due to some spooky configuration.
    #[cfg(not(esp32))]
    TIMG0::regs().rtccalicfg2().reset();

    TIMG0::regs()
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start().clear_bit());

    // Make sure we measure the crystal.
    #[cfg(soc_has_clock_node_timg0_function_clock)]
    {
        configure_timg0_function_clock(clocks, Timg0FunctionClockConfig::XtalClk);
        request_timg0_function_clock(clocks);
    }

    configure_timg0_calibration_clock(clocks, Timg0CalibrationClockConfig::RcSlowClk);
    request_timg0_calibration_clock(clocks);

    let calibration_clock_frequency = timg0_calibration_clock_frequency(clocks);

    TIMG0::regs().rtccalicfg().modify(|_, w| unsafe {
        w.rtc_cali_max().bits(SLOW_CLOCK_CYCLES as u16);
        w.rtc_cali_start_cycling().clear_bit();
        w.rtc_cali_start().set_bit()
    });

    // Delay, otherwise the CPU may read back the previous state of the completion flag and skip
    // waiting.
    ets_delay_us(SLOW_CLOCK_CYCLES * 1_000_000 / calibration_clock_frequency);

    // Wait for the calibration to finish
    while TIMG0::regs()
        .rtccalicfg()
        .read()
        .rtc_cali_rdy()
        .bit_is_clear()
    {}

    let cali_value = TIMG0::regs().rtccalicfg1().read().rtc_cali_value().bits();

    TIMG0::regs()
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start().clear_bit());
    release_timg0_calibration_clock(clocks);

    #[cfg(soc_has_clock_node_timg0_function_clock)]
    release_timg0_function_clock(clocks);

    let mhz = (cali_value * (calibration_clock_frequency / SLOW_CLOCK_CYCLES)) / 1_000_000;
    if mhz.abs_diff(40) < mhz.abs_diff(26) {
        XtalClkConfig::_40
    } else {
        XtalClkConfig::_26
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, config: XtalClkConfig) {
    // The stored configuration affects PLL settings instead. We save the value in a register
    // similar to ESP-IDF, just in case something relies on that, or, if we can in the future read
    // back the value instead of wasting RAM on it.

    // Used by `rtc_clk_xtal_freq_get` patched ROM function.
    let freq_mhz = config.value() / 1_000_000;
    LPWR::regs().store4().modify(|r, w| unsafe {
        // The data is stored in two copies of 16-bit values. The first bit overwrites the LSB of
        // the frequency value with DISABLE_ROM_LOG.

        // Copy the DISABLE_ROM_LOG bit
        let disable_rom_log_bit = r.bits() & Rtc::RTC_DISABLE_ROM_LOG;
        let half = (freq_mhz & (0xFFFF & !Rtc::RTC_DISABLE_ROM_LOG)) | disable_rom_log_bit;
        w.data().bits(half | (half << 16))
    });
}

// PLL_CLK

fn enable_pll_clk_impl(clocks: &mut ClockTree, en: bool) {
    const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
    const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;
    const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;

    // regi2c_ctrl_ll_i2c_bbpll_enable
    I2C_ANA_MST::regs()
        .ana_config()
        .modify(|_, w| w.bbpll_pd().bit(!en));

    LPWR::regs().options0().modify(|_, w| {
        let power_down = !en;
        w.bb_i2c_force_pd().bit(power_down);
        w.bbpll_force_pd().bit(power_down);
        w.bbpll_i2c_force_pd().bit(power_down)
    });

    if !en {
        return;
    }

    // Digital part

    SYSTEM::regs()
        .cpu_per_conf()
        .modify(|_, w| w.pll_freq_sel().set_bit()); // Undocumented, selects 480MHz PLL.

    // Analog part

    // Start BBPLL self-calibration
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().clear_bit();
        w.bbpll_stop_force_low().set_bit()
    });

    let div_ref: u8;
    let div7_0: u8;
    let dr1: u8;
    let dr3: u8;
    let dchgp: u8;
    let dcur: u8;
    let dbias: u8;
    // Configure 480M PLL
    match unwrap!(clocks.xtal_clk) {
        XtalClkConfig::_26 => {
            // Divide by 13 -> reference = 2MHz
            div_ref = 12;
            // Multiply by 236 + 4 -> PLL output = 480MHz
            div7_0 = 236;
            dr1 = 4;
            dr3 = 4;
            dchgp = 0;
            dcur = 0;
            dbias = 2;
        }
        XtalClkConfig::_40 => {
            // Divide by 1 -> reference = 40MHz
            div_ref = 0;
            // Multiply by 8 + 4 -> PLL output = 480MHz
            div7_0 = 8;
            dr1 = 0;
            dr3 = 0;
            dchgp = 5;
            dcur = 3;
            dbias = 2;
        }
    }

    regi2c::I2C_BBPLL_REG4.write_reg(0x6b);

    let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;
    let i2c_bbpll_dcur =
        (1 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (3 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

    regi2c::I2C_BBPLL_OC_REF.write_reg(i2c_bbpll_lref);
    regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(div7_0);
    regi2c::I2C_BBPLL_OC_DR1.write_field(dr1);
    regi2c::I2C_BBPLL_OC_DR3.write_field(dr3);
    regi2c::I2C_BBPLL_REG6.write_reg(i2c_bbpll_dcur);
    regi2c::I2C_BBPLL_OC_VCO_DBIAS.write_field(dbias);

    while I2C_ANA_MST::regs()
        .ana_conf0()
        .read()
        .bbpll_cal_done()
        .bit_is_clear()
    {}

    ets_delay_us(10);

    // Stop BBPLL self-calibration
    I2C_ANA_MST::regs().ana_conf0().modify(|_, w| {
        w.bbpll_stop_force_high().set_bit();
        w.bbpll_stop_force_low().clear_bit()
    });
}

// RC_FAST_CLK

fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, en: bool) {
    // XPD_RC_OSCILLATOR exists but we'll manage that separately
    const RTC_CNTL_FOSC_DFREQ_DEFAULT: u8 = 172;
    LPWR::regs().clk_conf().modify(|_, w| {
        // Confusing CK8M naming inherited from ESP32?

        // CK8M_DFREQ value controls tuning of 8M clock.
        unsafe { w.ck8m_dfreq().bits(RTC_CNTL_FOSC_DFREQ_DEFAULT) };

        w.enb_ck8m().bit(!en);
        w.dig_clk8m_en().bit(en); // digital system clock gate
        // Do not force the clock either way.
        w.ck8m_force_pd().clear_bit();
        w.ck8m_force_pu().clear_bit()
    });
    LPWR::regs()
        .timer1()
        .modify(|_, w| unsafe { w.ck8m_wait().bits(if en { 5 } else { 20 }) });
}

// OSC_SLOW_CLK

fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if !en {
        return;
    }

    // SCK_DCAP value controls tuning of 136k clock. The higher the value of DCAP, the lower the
    // frequency. There is no separate enable bit, just make sure the calibration value is set.
    const RTC_CNTL_SCK_DCAP_DEFAULT: u8 = 255;
    LPWR::regs()
        .rtc_cntl()
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

// RC_FAST_DIV_CLK

fn enable_rc_fast_div_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.enb_ck8m_div().bit(!en));
}

// SYSTEM_PRE_DIV_IN

// Not an actual MUX, used to allow configuring the DIV divider as one block.
// Related to CPU clock source configuration.
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

// CPU_PLL_DIV

fn enable_cpu_pll_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_pll_div_impl(_clocks: &mut ClockTree, _new_config: CpuPllDivConfig) {
    // Nothing to do.
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

// CRYPTO_CLK

fn enable_crypto_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_crypto_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<CryptoClkConfig>,
    _new_selector: CryptoClkConfig,
) {
    // Nothing to do, determined by CPU clock.
}

// MSPI_CLK

fn enable_mspi_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_mspi_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<MspiClkConfig>,
    _new_selector: MspiClkConfig,
) {
    // Nothing to do, determined by CPU clock.
}

// CPU_CLK

fn configure_cpu_clk_impl(
    clocks: &mut ClockTree,
    _old_selector: Option<CpuClkConfig>,
    new_selector: CpuClkConfig,
) {
    // Based on TRM Table 6.2-2
    if new_selector == CpuClkConfig::Pll {
        SYSTEM::regs().cpu_per_conf().modify(|_, w| unsafe {
            w.cpuperiod_sel().bits(match unwrap!(clocks.cpu_pll_div) {
                CpuPllDivConfig::_4 => 1,
                CpuPllDivConfig::_6 => 0,
            })
        });
    }

    SYSTEM::regs().sysclk_conf().modify(|_, w| unsafe {
        w.soc_clk_sel().bits(match new_selector {
            CpuClkConfig::Xtal => 0,
            CpuClkConfig::RcFast => 2,
            CpuClkConfig::Pll => 1,
        })
    });

    let apb_freq = Rate::from_hz(apb_clk_frequency(clocks));
    update_apb_frequency(apb_freq);

    let cpu_freq = Rate::from_hz(cpu_clk_frequency(clocks));
    ets_update_cpu_frequency_rom(cpu_freq.as_mhz());
}

fn update_apb_frequency(freq: Rate) {
    let freq_shifted = (freq.as_hz() >> 12) & 0xFFFF;
    let value = freq_shifted | (freq_shifted << 16);
    LPWR::regs()
        .store5()
        .modify(|_, w| unsafe { w.data().bits(value) });
}

// PLL_40M

fn enable_pll_40m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_60M

fn enable_pll_60m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_80M

fn enable_pll_80m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// CPU_DIV2

fn enable_cpu_div2_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// RC_FAST_CLK_DIV_N

fn enable_rc_fast_clk_div_n_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_rc_fast_clk_div_n_impl(_clocks: &mut ClockTree, new_config: RcFastClkDivNConfig) {
    let clk_conf = LPWR::regs().clk_conf();
    // Invalidate because we may be changing the divider from some other value
    let new_value = clk_conf.modify(|_, w| w.ck8m_div_sel_vld().clear_bit());
    // Update divider
    let new_value = clk_conf.write(|w| unsafe {
        w.bits(new_value);
        w.ck8m_div_sel().bits(new_config.value() as u8)
    });
    // Re-synchronize
    clk_conf.write(|w| {
        unsafe { w.bits(new_value) };
        w.ck8m_div_sel_vld().set_bit()
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
        // TODO: variants should be in PAC
        w.ana_clk_rtc_sel().bits(match new_selector {
            RtcSlowClkConfig::OscSlow => 1,
            RtcSlowClkConfig::RcSlow => 0,
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
    // TODO: variants should be fixed in PAC
    LPWR::regs().clk_conf().modify(|_, w| match new_selector {
        RtcFastClkConfig::Xtal => w.fast_clk_rtc_sel().clear_bit(),
        RtcFastClkConfig::Rc => w.fast_clk_rtc_sel().set_bit(),
    });
    ets_delay_us(3);
}

// LOW_POWER_CLK

fn enable_low_power_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing in esp-idf does this - is this managed by hardware, or the radio blobs?
    // SYSTEM::regs()
    //     .bt_lpck_div_frac()
    //     .modify(|_, w| w.lpclk_rtc_en().bit(en));
}

fn configure_low_power_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LowPowerClkConfig>,
    new_selector: LowPowerClkConfig,
) {
    SYSTEM::regs().bt_lpck_div_frac().modify(|_, w| {
        w.lpclk_sel_8m()
            .bit(new_selector == LowPowerClkConfig::RcFast);
        w.lpclk_sel_rtc_slow()
            .bit(new_selector == LowPowerClkConfig::RtcSlow);
        w.lpclk_sel_xtal()
            .bit(new_selector == LowPowerClkConfig::Xtal);
        w.lpclk_sel_xtal32k()
            .bit(new_selector == LowPowerClkConfig::OscSlow)
    });
}

fn enable_timg0_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    // TODO: should we model T0_DIVIDER, too?
    TIMG0::regs()
        .regclk()
        .modify(|_, w| w.timer_clk_is_active().bit(en));
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
            Timg0CalibrationClockConfig::RcSlowClk => 0,
            Timg0CalibrationClockConfig::RcFastDivClk => 1,
            Timg0CalibrationClockConfig::Osc32kClk => 2,
        })
    });
}

// TIMG0_WDT_CLOCK

fn enable_timg0_wdt_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // No separate clock control enable bit.
}

fn configure_timg0_wdt_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Timg0WdtClockConfig>,
    new_selector: Timg0WdtClockConfig,
) {
    TIMG0::regs().wdtconfig0().modify(|_, w| {
        w.wdt_use_xtal()
            .bit(new_selector == Timg0WdtClockConfig::XtalClk)
    });
}

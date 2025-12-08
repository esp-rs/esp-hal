//! Clock tree definitions and implementations for ESP32-C3.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (136k RC_SLOW and 17.5M RC_FAST) are not calibrated here, this system
//!   can only give a rough estimate of their frequency. They can be calibrated separately using a
//!   known crystal frequency.
//! - Some of the SOC capabilities are not implemented.
#![allow(dead_code, reason = "Some of this is bound to be unused")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use esp_rom_sys::rom::{ets_delay_us, ets_update_cpu_frequency_rom};

use crate::{
    peripherals::{APB_CTRL, I2C_ANA_MST, LPWR, SYSTEM, TIMG0, TIMG1},
    soc::regi2c,
    time::Rate,
};

define_clock_tree_types!();

// TODO: this should replace the current CpuClock enum. CpuClock is a bit of a misleading
// name as this will configure multiple things.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(
    clippy::enum_variant_names,
    reason = "MHz suffix indicates physical unit."
)]
#[non_exhaustive]
pub(crate) enum CpuClock {
    #[default]
    _80MHz,
    _160MHz,
    Custom(ClockConfig),
}

impl CpuClock {
    pub(crate) fn configure(self) {
        // Resolve presets
        //
        // The presets use 480MHz PLL by default, because that is the default value the chip boots
        // with, and changing it breaks USB Serial/JTAG.
        let mut config = match self {
            CpuClock::_80MHz => ClockConfig {
                xtal_clk: None,
                system_pre_div: None,
                pll_clk: Some(PllClkConfig::_480),
                cpu_pll_div_out: Some(CpuPllDivOutConfig::_80),
                cpu_clk: Some(CpuClkConfig::Pll),
                rc_fast_clk_div_n: Some(RcFastClkDivNConfig::new(0)),
                rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
                rtc_fast_clk: Some(RtcFastClkConfig::Rc),
                low_power_clk: Some(LowPowerClkConfig::RtcSlow),
            },
            CpuClock::_160MHz => ClockConfig {
                xtal_clk: None,
                system_pre_div: None,
                pll_clk: Some(PllClkConfig::_480),
                cpu_pll_div_out: Some(CpuPllDivOutConfig::_160),
                cpu_clk: Some(CpuClkConfig::Pll),
                rc_fast_clk_div_n: Some(RcFastClkDivNConfig::new(0)),
                rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
                rtc_fast_clk: Some(RtcFastClkConfig::Rc),
                low_power_clk: Some(LowPowerClkConfig::RtcSlow),
            },
            CpuClock::Custom(clock_config) => clock_config,
        };

        if config.xtal_clk.is_none() {
            // TODO: support multiple crystal frequencies (esp-idf supports 32M).
            config.xtal_clk = Some(XtalClkConfig::_40);
        }

        config.apply();
    }
}

// XTAL_CLK

fn configure_xtal_clk_impl(_clocks: &mut ClockTree, config: XtalClkConfig) {
    // The stored configuration affects PLL settings instead. We save the value in a register
    // similar to ESP-IDF, just in case something relies on that, or, if we can in the future read
    // back the value instead of wasting RAM on it.

    const DISABLE_ROM_LOG: u32 = 1;

    let freq_mhz = config.value() / 1_000_000;
    LPWR::regs().store4().modify(|r, w| unsafe {
        // The data is stored in two copies of 16-bit values. The first bit overwrites the LSB of
        // the frequency value with DISABLE_ROM_LOG.

        // Copy the DISABLE_ROM_LOG bit
        let disable_rom_log_bit = r.bits() & DISABLE_ROM_LOG;
        let half = (freq_mhz & (0xFFFF & !DISABLE_ROM_LOG)) | disable_rom_log_bit;
        w.data().bits(half | (half << 16))
    });
}

// PLL_CLK

fn enable_pll_clk_impl(clocks: &mut ClockTree, en: bool) {
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
    let pll_freq = unwrap!(clocks.pll_clk);
    let xtal_freq = unwrap!(clocks.xtal_clk);
    SYSTEM::regs()
        .cpu_per_conf()
        .modify(|_, w| w.pll_freq_sel().bit(pll_freq == PllClkConfig::_480));

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
    match pll_freq {
        PllClkConfig::_480 => {
            // Configure 480M PLL
            match xtal_freq {
                XtalClkConfig::_40 => {
                    div_ref = 0;
                    // Will multiply by 8 + 4 = 12
                    div7_0 = 8;
                    dr1 = 0;
                    dr3 = 0;
                    dchgp = 5;
                    dcur = 3;
                    dbias = 2;
                }
            }

            // Set the MODE_HF bit
            regi2c::I2C_BBPLL_REG4.write_reg(0x6b);
        }
        PllClkConfig::_320 => {
            // Configure 320M PLL
            match xtal_freq {
                XtalClkConfig::_40 => {
                    div_ref = 0;
                    // Will multiply by 4 + 4 = 8
                    div7_0 = 4;
                    dr1 = 0;
                    dr3 = 0;
                    dchgp = 5;
                    dcur = 3;
                    dbias = 2;
                }
            }

            // Clear the MODE_HF bit
            regi2c::I2C_BBPLL_REG4.write_reg(0x69);
        }
    }

    const I2C_BBPLL_OC_DCHGP_LSB: u32 = 4;
    const I2C_BBPLL_OC_DLREF_SEL_LSB: u32 = 6;
    const I2C_BBPLL_OC_DHREF_SEL_LSB: u32 = 4;

    let i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | div_ref;

    // Weird, that the last two writes flip these values...
    let i2c_bbpll_dcur =
        (2 << I2C_BBPLL_OC_DLREF_SEL_LSB) | (1 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

    regi2c::I2C_BBPLL_OC_REF.write_reg(i2c_bbpll_lref);
    regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(div7_0);
    regi2c::I2C_BBPLL_OC_DR1.write_field(dr1);
    regi2c::I2C_BBPLL_OC_DR3.write_field(dr3);
    regi2c::I2C_BBPLL_REG6.write_reg(i2c_bbpll_dcur);
    regi2c::I2C_BBPLL_OC_VCO_DBIAS.write_field(dbias);
    regi2c::I2C_BBPLL_OC_DHREF_SEL.write_field(2);
    regi2c::I2C_BBPLL_OC_DLREF_SEL.write_field(1);
}

fn configure_pll_clk_impl(_clocks: &mut ClockTree, _config: PllClkConfig) {
    // Nothing to do. The PLL may still be powered down. We'll configure it in
    // `enable_pll_clk_impl`.
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

// XTAL32K_CLK

fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, en: bool) {
    // RTCIO could be configured to allow an external oscillator to be used. We could model this
    // with a MUX, probably, but this is omitted for now for simplicity.

    const CLK_LL_XTAL_32K_DAC_VAL: u8 = 3;
    const CLK_LL_XTAL_32K_DRES_VAL: u8 = 3;
    const CLK_LL_XTAL_32K_DGM_VAL: u8 = 3;
    const CLK_LL_XTAL_32K_DBUF_VAL: bool = true; // differential buffer
    LPWR::regs().ext_xtl_conf().modify(|_, w| unsafe {
        w.xtal32k_gpio_sel().bit(false);

        w.dac_xtal_32k().bits(CLK_LL_XTAL_32K_DAC_VAL);
        w.dres_xtal_32k().bits(CLK_LL_XTAL_32K_DRES_VAL);
        w.dgm_xtal_32k().bits(CLK_LL_XTAL_32K_DGM_VAL);
        w.dbuf_xtal_32k().bit(CLK_LL_XTAL_32K_DBUF_VAL);

        w.xpd_xtal_32k().bit(en)
    });
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
    APB_CTRL::regs()
        .sysclk_conf()
        .modify(|_, w| unsafe { w.pre_div_cnt().bits(new_config.value() as u16 & 0x3FF) });
}

// CPU_PLL_DIV_OUT

fn enable_cpu_pll_div_out_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_pll_div_out_impl(_clocks: &mut ClockTree, _config: CpuPllDivOutConfig) {
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
    // Nothing to do.
}

// CPU_CLK

fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_cpu_clk_impl(
    clocks: &mut ClockTree,
    _old_selector: Option<CpuClkConfig>,
    new_selector: CpuClkConfig,
) {
    // Based on TRM Table 6.2-2
    if new_selector == CpuClkConfig::Pll {
        SYSTEM::regs().cpu_per_conf().modify(|_, w| unsafe {
            w.cpuperiod_sel()
                .bits(match unwrap!(clocks.cpu_pll_div_out) {
                    CpuPllDivOutConfig::_80 => 0,
                    CpuPllDivOutConfig::_160 => 1,
                })
        });
    }

    SYSTEM::regs().sysclk_conf().modify(|_, w| unsafe {
        w.pre_div_cnt().bits(0);
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

// PLL_80M

fn enable_pll_80m_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// PLL_160M

fn enable_pll_160m_impl(_clocks: &mut ClockTree, _en: bool) {
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
            RtcSlowClkConfig::Xtal32k => 1,
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
            .bit(new_selector == LowPowerClkConfig::Xtal32k)
    });
}

// TIMG0_FUNCTION_CLOCK

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
            Timg0CalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

// TIMG1_FUNCTION_CLOCK

fn enable_timg1_function_clock_impl(_clocks: &mut ClockTree, en: bool) {
    TIMG1::regs()
        .regclk()
        .modify(|_, w| w.timer_clk_is_active().bit(en));
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
            Timg0CalibrationClockConfig::RcSlowClk => 0,
            Timg0CalibrationClockConfig::RcFastDivClk => 1,
            Timg0CalibrationClockConfig::Xtal32kClk => 2,
        })
    });
}

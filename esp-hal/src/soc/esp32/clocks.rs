//! Clock tree definitions and implementations for ESP32.
//!
//! Remarks:
//! - Enabling a clock node assumes it has first been configured. Some fixed clock nodes don't need
//!   to be configured.
//! - Some information may be assumed, e.g. the possibility to disable watchdog timers before clock
//!   configuration.
//! - Internal RC oscillators (150k RC_SLOW and 8M RC_FAST) are not calibrated here, this system can
//!   only give a rough estimate of their frequency. They can be calibrated separately using a known
//!   crystal frequency.
//! - Some of the SOC capabilities are not implemented: using external 32K oscillator instead of a
//!   crystal, divider for CLK8M and possibly more.
#![allow(dead_code, reason = "Some of this is bound to be unused")]

// TODO: This is a temporary place for this, should probably be moved into clocks_ll.

use esp_rom_sys::rom::{ets_delay_us, ets_update_cpu_frequency_rom};

use crate::{
    efuse::{Efuse, VOL_LEVEL_HP_INV},
    peripherals::{APB_CTRL, DPORT, LPWR, RTC_IO, TIMG0, TIMG1},
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

    /// 160 MHz CPU clock
    _160MHz = 160,

    /// 240 MHz CPU clock
    _240MHz = 240,
}

impl CpuClock {
    const PRESET_80: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_320),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::_4),
        syscon_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
    };
    const PRESET_160: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_320),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::_2),
        syscon_pre_div: None,
        cpu_clk: Some(CpuClkConfig::Pll),
        rtc_slow_clk: Some(RtcSlowClkConfig::RcSlow),
        rtc_fast_clk: Some(RtcFastClkConfig::Rc),
    };
    const PRESET_240: ClockConfig = ClockConfig {
        xtal_clk: None,
        pll_clk: Some(PllClkConfig::_480),
        apll_clk: None,
        cpu_pll_div: Some(CpuPllDivConfig::_2),
        syscon_pre_div: None,
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
        // Detect XTAL if unset.
        // FIXME: this doesn't support running from RC_FAST_CLK. We should rework detection to only
        // run when requesting XTAL.
        ClockTree::with(|clocks| {
            if self.xtal_clk.is_none() {
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
    configure_syscon_pre_div(clocks, SysconPreDivConfig::new(0));
    configure_cpu_clk(clocks, CpuClkConfig::Xtal);

    // Make sure the process doesn't time out due to some spooky configuration.
    #[cfg(not(esp32))]
    TIMG0::regs().rtccalicfg2().reset();

    TIMG0::regs()
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start().clear_bit());

    configure_timg0_calibration_clock(clocks, Timg0CalibrationClockConfig::default());
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

    let mhz = (cali_value * (calibration_clock_frequency / SLOW_CLOCK_CYCLES)) / 1_000_000;
    if mhz.abs_diff(40) < mhz.abs_diff(26) {
        XtalClkConfig::_40
    } else {
        XtalClkConfig::_26
    }
}

const BBPLL_IR_CAL_DELAY_VAL: u8 = 0x18;
const BBPLL_IR_CAL_EXT_CAP_VAL: u8 = 0x20;
const BBPLL_OC_ENB_FCAL_VAL: u8 = 0x9a;
const BBPLL_OC_ENB_VCON_VAL: u8 = 0x00;
const BBPLL_BBADC_CAL_7_0_VAL: u8 = 0x00;

const RTC_CNTL_DBIAS_1V10: u8 = 4;
const RTC_CNTL_DBIAS_1V25: u8 = 7;

const BBPLL_ENDIV5_VAL_320M: u8 = 0x43;
const BBPLL_BBADC_DSMP_VAL_320M: u8 = 0x84;
const BBPLL_ENDIV5_VAL_480M: u8 = 0xc3;
const BBPLL_BBADC_DSMP_VAL_480M: u8 = 0x74;

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
    // TODO: this should probably be refcounted with APLL, but ESP32 TRM states APLL_CLK is sourced
    // from PLL. Currently it's unclear how these relate to each other - but the internal I2C bus is
    // required for both of them.
    LPWR::regs().options0().modify(|_, w| {
        let power_down = !en;
        w.bias_i2c_force_pd().bit(power_down);
        w.bb_i2c_force_pd().bit(power_down);
        w.bbpll_force_pd().bit(power_down);
        w.bbpll_i2c_force_pd().bit(power_down)
    });

    if !en {
        return;
    }

    let xtal_cfg = unwrap!(clocks.xtal_clk);
    let pll_cfg = unwrap!(clocks.pll_clk);

    // This classification is arbitrary based on variable names and what (PLL output or XTAL input)
    // affects the values.
    struct PllParams {
        // The only parameter I could infer, it divides the reference clock (XTAL).
        // We'll either use a reference of 40MHz or 2MHz (or maybe 20/1MHz).
        // The rest of the parameters somehow ensure this divided reference clock is multiplied to
        // the PLL target frequency.
        div_ref: u8,
        div10_8: u8,
        lref: u8,
        dcur: u8,
        bw: u8,
        div7_0: u8,
    }
    impl PllParams {
        fn apply(&self) {
            regi2c::I2C_BBPLL_OC_LREF
                .write_reg((self.lref << 7) | (self.div10_8 << 4) | self.div_ref);
            regi2c::I2C_BBPLL_OC_DIV_REG.write_reg(self.div7_0);
            regi2c::I2C_BBPLL_OC_DCUR.write_reg((self.bw << 6) | self.dcur);
        }
    }

    // maybe?
    struct VoltageParams {
        endiv5: u8,
        bbadc_dsmp: u8,
        dbias: u8,
    }
    impl VoltageParams {
        fn apply(&self) {
            LPWR::regs()
                .reg()
                .modify(|_, w| unsafe { w.dig_dbias_wak().bits(self.dbias) });

            regi2c::I2C_BBPLL_ENDIV5.write_reg(self.endiv5);
            regi2c::I2C_BBPLL_BBADC_DSMP.write_reg(self.bbadc_dsmp);
        }
    }

    let voltage_params = match pll_cfg {
        PllClkConfig::_320 => VoltageParams {
            dbias: RTC_CNTL_DBIAS_1V10,
            endiv5: BBPLL_ENDIV5_VAL_320M,
            bbadc_dsmp: BBPLL_BBADC_DSMP_VAL_320M,
        },
        PllClkConfig::_480 => VoltageParams {
            dbias: RTC_CNTL_DBIAS_1V25 - Efuse::read_field_le::<u8>(VOL_LEVEL_HP_INV),
            endiv5: BBPLL_ENDIV5_VAL_480M,
            bbadc_dsmp: BBPLL_BBADC_DSMP_VAL_480M,
        },
    };

    let pll_params = match xtal_cfg {
        XtalClkConfig::_40 => PllParams {
            div_ref: 0, // divided ref ~~ 40MHz
            lref: 0,
            dcur: 6,
            bw: 3,
            div10_8: 0,
            div7_0: match pll_cfg {
                PllClkConfig::_320 => 32,
                PllClkConfig::_480 => 28,
            },
        },

        XtalClkConfig::_26 => PllParams {
            div_ref: 12, // divided ref ~~ 2MHz
            lref: 1,
            dcur: 0,
            bw: 1,
            div10_8: 4,
            div7_0: match pll_cfg {
                PllClkConfig::_320 => 224,
                PllClkConfig::_480 => 144,
            },
        },
    };

    voltage_params.apply();
    pll_params.apply();

    // Unclear why in esp-idf this depends on the source of RTC_SLOW_CLK. Let's just assume the
    // slowest option.
    const SOC_DELAY_PLL_ENABLE_WITH_32K: u32 = 160;
    ets_delay_us(SOC_DELAY_PLL_ENABLE_WITH_32K);
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
    const RTC_CNTL_CK8M_DFREQ_DEFAULT: u8 = 172;
    LPWR::regs().clk_conf().modify(|_, w| {
        // Set divider = 0 so that RC_FAST_CLK is 8MHz.
        unsafe {
            w.ck8m_div_sel().bits(0);
            // CK8M_DFREQ value controls tuning of 8M clock.
            w.ck8m_dfreq().bits(RTC_CNTL_CK8M_DFREQ_DEFAULT);
        }

        let power_down = !en;
        w.ck8m_force_pd().bit(power_down);
        w.ck8m_force_pu().bit(!power_down)
    });
}

// PLL_F160M_CLK

fn enable_pll_f160m_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

// CPU_PLL_DIV_IN

// Not an actual MUX, used to allow configuring the DIVA divider as one block.
// Related to CPU clock source configuration.
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

// SYSCON_PRE_DIV_IN

// Not an actual MUX, used to allow configuring the DIVB divider as one block.
// Related to CPU clock source configuration.
fn enable_syscon_pre_div_in_impl(_: &mut ClockTree, _: bool) {
    // Nothing to do.
}

fn configure_syscon_pre_div_in_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<SysconPreDivInConfig>,
    _new_selector: SysconPreDivInConfig,
) {
    // Nothing to do.
}

// SYSCON_PRE_DIV

fn enable_syscon_pre_div_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_syscon_pre_div_impl(_clocks: &mut ClockTree, new_config: SysconPreDivConfig) {
    APB_CTRL::regs()
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
    APB_CTRL::regs()
        .xtal_tick_conf()
        .write(|w| unsafe { w.xtal_tick_num().bits(new_config.value() as u8) });
}

// REF_TICK_FOSC

fn enable_ref_tick_fosc_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_fosc_impl(_clocks: &mut ClockTree, new_config: RefTickFoscConfig) {
    APB_CTRL::regs()
        .ck8m_tick_conf()
        .write(|w| unsafe { w.ck8m_tick_num().bits(new_config.value() as u8) });
}

// REF_TICK_APLL

fn enable_ref_tick_apll_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_apll_impl(_clocks: &mut ClockTree, new_config: RefTickApllConfig) {
    APB_CTRL::regs()
        .apll_tick_conf()
        .write(|w| unsafe { w.apll_tick_num().bits(new_config.value() as u8) });
}

// REF_TICK_PLL

fn enable_ref_tick_pll_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_ref_tick_pll_impl(_clocks: &mut ClockTree, new_config: RefTickPllConfig) {
    APB_CTRL::regs()
        .pll_tick_conf()
        .write(|w| unsafe { w.pll_tick_num().bits(new_config.value() as u8) });
}

// CPU_CLK

fn configure_cpu_clk_impl(
    clocks: &mut ClockTree,
    _old_selector: Option<CpuClkConfig>,
    new_selector: CpuClkConfig,
) {
    // Based on TRM Table 7.2-2

    // Configure CPUPERIOD divider(?) to output(?) 80, 160 or 240MHz, although because APLL is
    // freely configurable the final clock can be different.
    let pll_selector = clocks.cpu_pll_div_in;
    let bbpll_config = clocks.pll_clk.map(|pll| pll.value()).unwrap_or_default();
    let is_240mhz = pll_selector == Some(CpuPllDivInConfig::Pll) && bbpll_config == 480_000_000;
    let clock_source_sel1_bit = match clocks.cpu_pll_div {
        Some(CpuPllDivConfig::_2) => {
            if is_240mhz {
                2
            } else {
                1
            }
        }
        _ => 0, // divide-by-4 or don't care
    };

    DPORT::regs()
        .cpu_per_conf()
        .modify(|_, w| unsafe { w.cpuperiod_sel().bits(clock_source_sel1_bit) });

    LPWR::regs().clk_conf().modify(|_, w| match new_selector {
        CpuClkConfig::Xtal => w.soc_clk_sel().xtal(),
        CpuClkConfig::RcFast => w.soc_clk_sel().ck8m(),
        CpuClkConfig::Apll => w.soc_clk_sel().apll(),
        CpuClkConfig::Pll => w.soc_clk_sel().pll(),
    });

    // Store frequencies in expected places.
    let cpu_freq = Rate::from_hz(cpu_clk_frequency(clocks));
    ets_update_cpu_frequency_rom(cpu_freq.as_mhz());

    let apb_freq = Rate::from_hz(apb_clk_frequency(clocks));
    update_apb_frequency(apb_freq);
}

fn update_apb_frequency(freq: Rate) {
    let freq_shifted = (freq.as_hz() >> 12) & 0xFFFF;
    let value = freq_shifted | (freq_shifted << 16);
    LPWR::regs()
        .store5()
        .modify(|_, w| unsafe { w.data().bits(value) });
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
    // RTCIO could be configured to allow an external oscillator to be used. We could model this
    // with a MUX, probably, but this is omitted for now for simplicity.

    // TODO: implement esp-idf's CONFIG_RTC_EXT_CRYST_ADDIT_CURRENT options.
    const CLK_LL_XTAL_32K_DAC_VAL: u8 = 1;
    const CLK_LL_XTAL_32K_DRES_VAL: u8 = 3;
    const CLK_LL_XTAL_32K_DBIAS_VAL: u8 = 0;
    RTC_IO::regs().xtal_32k_pad().modify(|_, w| unsafe {
        w.dac_xtal_32k().bits(CLK_LL_XTAL_32K_DAC_VAL);
        w.dres_xtal_32k().bits(CLK_LL_XTAL_32K_DRES_VAL);
        w.dbias_xtal_32k().bits(CLK_LL_XTAL_32K_DBIAS_VAL);

        if en {
            w.x32n_rde().clear_bit();
            w.x32n_rue().clear_bit();
            w.x32n_fun_ie().clear_bit();

            w.x32p_rde().clear_bit();
            w.x32p_rue().clear_bit();
            w.x32p_fun_ie().clear_bit();
        }

        w.x32n_mux_sel().bit(en);
        w.x32p_mux_sel().bit(en);
        w.xpd_xtal_32k().bit(en)
    });

    // Enable for digital part
    LPWR::regs()
        .clk_conf()
        .modify(|_, w| w.dig_xtal32k_en().bit(en));
}

// RC_SLOW_CLK

fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, en: bool) {
    if en {
        // SCK_DCAP value controls tuning of 150k clock. The higher the value of DCAP, the lower the
        // frequency. There is no separate enable bit, just make sure the calibration value is set.
        const RTC_CNTL_SCK_DCAP_DEFAULT: u8 = 255;
        LPWR::regs()
            .reg()
            .modify(|_, w| unsafe { w.sck_dcap().bits(RTC_CNTL_SCK_DCAP_DEFAULT) });
    }
}

// RC_FAST_DIV_CLK

fn enable_rc_fast_div_clk_impl(_clocks: &mut ClockTree, en: bool) {
    LPWR::regs().clk_conf().modify(|_, w| {
        w.enb_ck8m_div().bit(en);
        w.ck8m_div().div256()
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
    LPWR::regs().clk_conf().modify(|_, w| match new_selector {
        RtcSlowClkConfig::RcSlow => w.ana_clk_rtc_sel().slow_ck(),
        RtcSlowClkConfig::Xtal => w.ana_clk_rtc_sel().ck_xtal_32k(),
        RtcSlowClkConfig::RcFast => w.ana_clk_rtc_sel().ck8m_d256_out(),
    });
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
    LPWR::regs().clk_conf().modify(|_, w| match new_selector {
        RtcFastClkConfig::Xtal => w.fast_clk_rtc_sel().xtal_div_4(),
        RtcFastClkConfig::Rc => w.fast_clk_rtc_sel().ck8m(),
    });
}

// MCPWM0_FUNCTION_CLOCK

fn enable_mcpwm0_function_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_mcpwm0_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Mcpwm0FunctionClockConfig>,
    _new_selector: Mcpwm0FunctionClockConfig,
) {
    // Nothing to do.
}

// MCPWM1_FUNCTION_CLOCK

fn enable_mcpwm1_function_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_mcpwm1_function_clock_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<Mcpwm0FunctionClockConfig>,
    _new_selector: Mcpwm0FunctionClockConfig,
) {
    // Nothing to do.
}

// TIMG0_CALIBRATION_CLOCK

fn enable_timg0_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do, calibration clocks can only be selected. They are gated by the CALI_START bit,
    // which is managed by the calibration process.
}

impl Timg0CalibrationClockConfig {
    fn cali_clk_sel_bits(self) -> u8 {
        match self {
            Timg0CalibrationClockConfig::RcSlowClk => 0,
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

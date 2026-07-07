//! Clock tree for ESP32-P4X (chip revision v3.x / eco5).
//!
//! CPLL: 400 MHz (eco5 default), 360 MHz (conservative)
//! SPLL: 480 MHz (peripheral clocks)
//! MPLL: 400 MHz (PSRAM, media)
//! XTAL: 40 MHz
//!
//! Clock hierarchy: XTAL -> CPLL -> CPU_ROOT -> CPU/APB dividers
//! SPLL -> PLL_F240M/160M/120M/80M/20M (peripheral clocks)
#![allow(dead_code, reason = "Clock functions called from generated macro code")]
#![allow(
    missing_docs,
    reason = "Clock-tree types come from a generated macro that does not emit doc strings"
)]
#![allow(
    clippy::enum_variant_names,
    reason = "CPU frequency variant names follow the chip-spec MHz convention"
)]

use esp_rom_sys::rom::ets_update_cpu_frequency_rom;

use crate::{
    peripherals::{HP_SYS_CLKRST, LP_AON_CLKRST, PMU},
    soc::regi2c,
    time::Rate,
};

define_clock_tree_types!();

/// CPU clock frequency presets for ESP32-P4.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum CpuClock {
    /// 400 MHz CPU clock (eco5 / v3.x maximum)
    /// CPLL -> CPU_ROOT -> CPU/1, APB divider /4 = 100MHz
    _400MHz = 400,

    /// 200 MHz CPU clock (low power)
    /// CPLL -> CPU_ROOT -> CPU/2, APB /2 = 100MHz
    _200MHz = 200,

    /// 100 MHz CPU clock (ultra low power)
    /// CPLL -> CPU_ROOT -> CPU/4, APB /1 = 100MHz
    #[default]
    _100MHz = 100,
}

impl CpuClock {
    // Preset: 400 MHz CPU, 100 MHz APB
    const PRESET_400: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpll_clk: Some(CpllClkConfig::_400),
        spll_clk: Some(SpllClkConfig::_480),
        mpll_clk: Some(MpllClkConfig::_500),
        cpu_clk: Some(CpuClkConfig::new(0)), // /1 = 400 MHz
        mem_clk: Some(MemClkConfig::new(1)), // /2 = 200 MHz
        sys_clk: Some(SysClkConfig::new(0)), // /1 = 200 MHz
        apb_clk: Some(ApbClkConfig::new(1)), // /2 = 100 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };

    // TODO: 360 MHz preset

    const PRESET_200: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpll_clk: Some(CpllClkConfig::_400),
        spll_clk: Some(SpllClkConfig::_480),
        mpll_clk: Some(MpllClkConfig::_500),
        cpu_clk: Some(CpuClkConfig::new(1)), // /2 = 200 MHz
        mem_clk: Some(MemClkConfig::new(0)), // /1 = 200 MHz
        sys_clk: Some(SysClkConfig::new(0)), // /1 = 200 MHz
        apb_clk: Some(ApbClkConfig::new(1)), // /2 = 100 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };

    const PRESET_100: ClockConfig = ClockConfig {
        cpu_root_clk: Some(CpuRootClkConfig::Cpll),
        cpll_clk: Some(CpllClkConfig::_400),
        spll_clk: Some(SpllClkConfig::_480),
        mpll_clk: Some(MpllClkConfig::_500),
        cpu_clk: Some(CpuClkConfig::new(3)), // /4 = 100 MHz
        mem_clk: Some(MemClkConfig::new(0)), // /1 = 100 MHz
        sys_clk: Some(SysClkConfig::new(0)), // /1 = 100 MHz
        apb_clk: Some(ApbClkConfig::new(0)), // /1 = 100 MHz
        lp_fast_clk: Some(LpFastClkConfig::RcFast),
        lp_slow_clk: Some(LpSlowClkConfig::RcSlow),
        timg_calibration_clock: None,
    };
}

impl From<CpuClock> for ClockConfig {
    fn from(value: CpuClock) -> ClockConfig {
        match value {
            CpuClock::_400MHz => CpuClock::PRESET_400,
            CpuClock::_200MHz => CpuClock::PRESET_200,
            CpuClock::_100MHz => CpuClock::PRESET_100,
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
            v if v == CpuClock::PRESET_400 => Some(CpuClock::_400MHz),
            v if v == CpuClock::PRESET_200 => Some(CpuClock::_200MHz),
            v if v == CpuClock::PRESET_100 => Some(CpuClock::_100MHz),
            _ => None,
        }
    }

    pub(crate) fn configure(self, clocks: &mut ClockTree) {
        self.apply(clocks);
    }
}

// CPLL_CLK

fn enable_cpll_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // PLLs are always on, for now
}

fn configure_cpll_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpllClkConfig>,
    new_config: CpllClkConfig,
) {
    // div7_0 = (freq_mhz / 40) - 1
    let div7_0: u8 = match new_config {
        CpllClkConfig::_360 => 9,
        CpllClkConfig::_400 => 10,
    };

    let lref: u8 = 0x50; // dchgp=5, div_ref=0, oc_enb_fcal=0
    let dcur: u8 = 0x73; // dlref_sel=1, dhref_sel=3, dcur=3

    regi2c::I2C_CPLL_OC_REF_DIV.write_reg(lref);
    regi2c::I2C_CPLL_OC_DIV_7_0.write_reg(div7_0);
    regi2c::I2C_CPLL_OC_DCUR.write_reg(dcur);

    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().clear_bit());

    // Wait for calibration to complete
    while !HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .cpu_pll_cal_end()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }

    // Stop calibration: set cpu_pll_cal_stop = 1
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.cpu_pll_cal_stop().set_bit());

    // Small delay for PLL to stabilize
    crate::rom::ets_delay_us(10);
}

// SPLL_CLK

fn enable_spll_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // PLLs are always on, for now
}

fn configure_spll_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<SpllClkConfig>,
    new_config: SpllClkConfig,
) {
    // div7_0 = (freq_mhz / 40) - 1
    let div7_0: u8 = match new_config {
        SpllClkConfig::_480 => 11,
        SpllClkConfig::_240 => 5,
    };

    // Same OC_REF_DIV and OC_DCUR values as CPLL
    let lref: u8 = 0x50; // dchgp=5, div_ref=0, oc_enb_fcal=0
    let dcur: u8 = 0x73; // dlref_sel=1, dhref_sel=3, dcur=3

    regi2c::I2C_SPLL_OC_REF_DIV.write_reg(lref);
    regi2c::I2C_SPLL_OC_DIV_7_0.write_reg(div7_0);
    regi2c::I2C_SPLL_OC_DCUR.write_reg(dcur);

    // Run SPLL calibration
    //      HP_SYS_CLKRST.ana_pll_ctrl0.sys_pll_cal_stop
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.sys_pll_cal_stop().clear_bit());

    while !HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .sys_pll_cal_end()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }

    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.sys_pll_cal_stop().set_bit());

    crate::rom::ets_delay_us(10);
}

// MPLL_CLK

fn enable_mpll_clk_impl(clocks: &mut ClockTree, en: bool) {
    PMU::regs().rf_pwc().modify(|_, w| w.mspi_phy_xpd().bit(en));
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| w.lp_aonclkrst_hp_mpll_500m_clk_en().bit(en));

    if !en {
        return;
    }

    regi2c::I2C_MPLL_DHREF_DHREF.write_field(3);

    let rstb = regi2c::I2C_MPLL_IR_CAL_RSTB.read();
    regi2c::I2C_MPLL_IR_CAL_RSTB.write_reg(rstb & 0xDF);
    regi2c::I2C_MPLL_IR_CAL_RSTB.write_reg(rstb | (1 << 5));

    // div = freq_mhz/20 - 1, ref_div = 1 -> MPLL = XTAL(40) * (div+1) / (ref_div+1)
    let ref_div: u8 = 1;
    let div: u8 = match unwrap!(clocks.mpll_clk) {
        MpllClkConfig::_320 => 15,
        MpllClkConfig::_400 => 19,
        MpllClkConfig::_500 => 24,
    };
    let div_val: u8 = (div << 3) | ref_div;
    regi2c::I2C_MPLL_DIV_REG_ADDR.write_reg(div_val);

    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().clear_bit());

    while HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .mspi_cal_end()
        .bit_is_clear()
    {
        core::hint::spin_loop();
    }
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().set_bit());

    crate::rom::ets_delay_us(10);
}

fn configure_mpll_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<MpllClkConfig>,
    _new_config: MpllClkConfig,
) {
    // Configuration applied in enable_mpll_clk_impl
}

// CPU_ROOT_CLK (mux: XTAL / CPLL / RC_FAST)
fn configure_cpu_root_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<CpuRootClkConfig>,
    new_selector: CpuRootClkConfig,
) {
    //      LP_AON_CLKRST.hp_clk_ctrl.hp_root_clk_src_sel: 0=XTAL, 1=CPLL, 2=RC_FAST
    let sel = match new_selector {
        CpuRootClkConfig::Xtal => 0,
        CpuRootClkConfig::Cpll => 1,
        CpuRootClkConfig::RcFast => 2,
    };
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hp_clk_ctrl()
        .modify(|_, w| unsafe { w.lp_aonclkrst_hp_root_clk_src_sel().bits(sel) });
}

// CPU_CLK divider
fn enable_cpu_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<CpuClkConfig>,
    new_config: CpuClkConfig,
) {
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .modify(|_, w| unsafe {
            w.cpu_clk_div_num().bits(new_config.divisor as u8);
            w.cpu_clk_div_numerator().bits(0);
            w.cpu_clk_div_denominator().bits(0)
        });

    // Trigger divider update
    update_divider();

    let cpu_freq = Rate::from_hz(cpu_clk_frequency());
    ets_update_cpu_frequency_rom(cpu_freq.as_mhz());
}

fn update_divider() {
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .modify(|_, w| w.soc_clk_div_update().set_bit());
    while HP_SYS_CLKRST::regs()
        .root_clk_ctrl0()
        .read()
        .soc_clk_div_update()
        .bit_is_set()
    {
        core::hint::spin_loop();
    }
}

// MEM_CLK

fn enable_mem_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_mem_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<MemClkConfig>,
    new_config: MemClkConfig,
) {
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl1()
        .modify(|_, w| unsafe {
            w.mem_clk_div_num().bits(new_config.divisor as u8);
            w.mem_clk_div_numerator().bits(0);
            w.mem_clk_div_denominator().bits(0)
        });

    // Trigger divider update
    update_divider();
}

// SYS_CLK

fn enable_sys_clk_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do.
}

fn configure_sys_clk_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<SysClkConfig>,
    new_config: SysClkConfig,
) {
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl1()
        .modify(|_, w| unsafe { w.sys_clk_div_num().bits(new_config.divisor as u8) });
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl2()
        .modify(|_, w| unsafe {
            w.sys_clk_div_numerator().bits(0);
            w.sys_clk_div_denominator().bits(0)
        });

    // Trigger divider update
    update_divider();
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
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl2()
        .modify(|_, w| unsafe {
            w.apb_clk_div_num().bits(new_config.divisor as u8);
            w.apb_clk_div_numerator().bits(0)
        });
    HP_SYS_CLKRST::regs()
        .root_clk_ctrl3()
        .modify(|_, w| unsafe { w.apb_clk_div_denominator().bits(0) });

    // Trigger divider update
    update_divider();
}

// LP_FAST_CLK mux
fn configure_lp_fast_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpFastClkConfig>,
    new_selector: LpFastClkConfig,
) {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_lp_clk_conf()
        .modify(|_, w| unsafe {
            w.lp_aonclkrst_fast_clk_sel().bits(match new_selector {
                LpFastClkConfig::RcFast => 0,
                LpFastClkConfig::XtalD2 => 1,
            })
        });
}

// LP_SLOW_CLK mux
fn configure_lp_slow_clk_impl(
    _clocks: &mut ClockTree,
    _old_selector: Option<LpSlowClkConfig>,
    new_selector: LpSlowClkConfig,
) {
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_lp_clk_conf()
        .modify(|_, w| unsafe {
            w.lp_aonclkrst_slow_clk_sel().bits(match new_selector {
                LpSlowClkConfig::RcSlow => 0,
                LpSlowClkConfig::Xtal32k => 1,
                // LpSlowClkConfig::Rc32k => 2,
                LpSlowClkConfig::OscSlow => 3,
            })
        });
}

// Per-instance clock impl for UART (called on UartInstance enum)

impl UartInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // UART function clock enable is handled by peripheral clock gates in system.rs
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartFunctionClockConfig>,
        _new_config: UartFunctionClockConfig,
    ) {
        // TODO: Configure UART clock source selection
        // HP_SYS_CLKRST PERI_CLK_CTRL110-114 for UART0-4
    }

    fn enable_baud_rate_generator_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // Baud rate generator is always on when UART is enabled
    }

    fn configure_baud_rate_generator_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<UartBaudRateGeneratorConfig>,
        _new_config: UartBaudRateGeneratorConfig,
    ) {
        // Baud rate is configured directly in UART registers, not here
    }
}

// Per-instance clock impl for TIMG

impl TimgInstance {
    fn enable_function_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {
        // TIMG function clock is managed by peripheral clock gates
    }

    fn configure_function_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgFunctionClockConfig>,
        _new_config: TimgFunctionClockConfig,
    ) {
        // TODO: Configure TIMG clock source
        // HP_SYS_CLKRST PERI_CLK_CTRL20/21
    }

    fn enable_wdt_clock_impl(self, _clocks: &mut ClockTree, _en: bool) {}

    fn configure_wdt_clock_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<TimgWdtClockConfig>,
        _new_config: TimgWdtClockConfig,
    ) {
        // TODO: Configure TIMG WDT clock source
    }
}

// System clock impl functions

// Mux enable stubs (mux nodes need enable functions too)
fn enable_cpu_root_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_lp_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_lp_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}

// Source clock enable/disable stubs (PLLs, oscillators)
fn enable_rc_fast_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_xtal32k_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_osc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_rc_slow_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_rc32k_clk_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f20m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f80m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f120m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f160m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f240m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f25m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_pll_f50m_impl(_clocks: &mut ClockTree, _en: bool) {}
fn enable_xtal_d2_clk_impl(_clocks: &mut ClockTree, _en: bool) {}

// TIMG_CALIBRATION_CLOCK

fn enable_timg_calibration_clock_impl(_clocks: &mut ClockTree, _en: bool) {
    // Nothing to do here.
}

impl MipiDsiInstance {
    // MIPI_DSI_DPI_CLK
    // HP_SYS_CLKRST.peri_clk_ctrl03: mipi_dsi_dpiclk_{src_sel,div_num,en}

    fn enable_dpi_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_dsi_dpiclk_en().bit(en));
    }

    fn configure_dpi_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<MipiDsiDpiClkConfig>,
        new_config: MipiDsiDpiClkConfig,
    ) {
        // Register values: 0 = XTAL, 1 = PLL_F240M, 2 = PLL_F160M.
        let src_sel = match new_config.sclk() {
            MipiDsiDpiClkSclk::Xtal => 0,
            MipiDsiDpiClkSclk::PllF240m => 1,
            MipiDsiDpiClkSclk::PllF160m => 2,
        };
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| unsafe {
                w.mipi_dsi_dpiclk_src_sel().bits(src_sel);
                w.mipi_dsi_dpiclk_div_num().bits(new_config.div_num() as u8)
            });
    }

    // MIPI_DSI_PHY_PLL_REFCLK
    // HP_SYS_CLKRST.peri_clk_ctrl03: mipi_dsi_dphy_pll_refclk_{src_sel,div_num,en}

    fn enable_phy_pll_refclk_impl(self, _clocks: &mut ClockTree, en: bool) {
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_dsi_dphy_pll_refclk_en().bit(en));
    }

    fn configure_phy_pll_refclk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<MipiDsiPhyPllRefclkConfig>,
        new_config: MipiDsiPhyPllRefclkConfig,
    ) {
        // Hardware register: 0=XTAL, 1=APLL (not modelled), 2=CPLL, 3=SPLL, 4=MPLL.
        let src_sel = match new_config.sclk() {
            MipiDsiPhyPllRefclkSclk::Xtal => 0u8,
            MipiDsiPhyPllRefclkSclk::Cpll => 2,
            MipiDsiPhyPllRefclkSclk::Spll => 3,
            MipiDsiPhyPllRefclkSclk::Mpll => 4,
        };
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| unsafe {
                w.mipi_dsi_dphy_pll_refclk_src_sel().bits(src_sel);
                w.mipi_dsi_dphy_pll_refclk_div_num()
                    .bits(new_config.div_num() as u8)
            });
    }

    // MIPI_DSI_PHY_CFG_CLK
    // HP_SYS_CLKRST.peri_clk_ctrl02: mipi_dsi_dphy_clk_src_sel
    // HP_SYS_CLKRST.peri_clk_ctrl03: mipi_dsi_dphy_cfg_clk_en

    fn enable_phy_cfg_clk_impl(self, _clocks: &mut ClockTree, en: bool) {
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_dsi_dphy_cfg_clk_en().bit(en));
    }

    fn configure_phy_cfg_clk_impl(
        self,
        _clocks: &mut ClockTree,
        _old_config: Option<MipiDsiPhyCfgClkConfig>,
        new_config: MipiDsiPhyCfgClkConfig,
    ) {
        // Register values: 0 = PLL_F20M, 1 = RC_FAST, 2 = PLL_F25M.
        let src_sel = match new_config {
            MipiDsiPhyCfgClkConfig::PllF20m => 0,
            MipiDsiPhyCfgClkConfig::RcFast => 1,
            MipiDsiPhyCfgClkConfig::PllF25m => 2,
        };
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl02()
            .modify(|_, w| unsafe { w.mipi_dsi_dphy_clk_src_sel().bits(src_sel) });
    }
}

fn configure_timg_calibration_clock_impl(
    _clocks: &mut ClockTree,
    _old_config: Option<TimgCalibrationClockConfig>,
    new_config: TimgCalibrationClockConfig,
) {
    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl21()
        .modify(|_, w| unsafe {
            w.timergrp0_tgrt_clk_src_sel().bits(match new_config {
                TimgCalibrationClockConfig::MpllClk => 0,
                TimgCalibrationClockConfig::SpllClk => 1,
                TimgCalibrationClockConfig::CpllClk => 2,
                TimgCalibrationClockConfig::RcFastClk => 7,
                TimgCalibrationClockConfig::RcSlowClk => 8,
                TimgCalibrationClockConfig::Xtal32kClk => 10,
            })
        });
}

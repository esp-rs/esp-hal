#![cfg_attr(docsrs, procmacros::doc_replace)]
//! MIPI DSI bus driver (ESP32-P4).
//!
//! Wraps `MIPI_DSI_HOST` and `MIPI_DSI_BRIDGE` register blocks behind two
//! virtual-peripheral ownership tokens (`MIPI_DSI` + `VDMA`).
//!
//! # Example
//!
//! ```rust,no_run
//! # {before_snippet}
//! use esp_hal::mipi_dsi::{Config, DataLanes, MipiDsi, PhyPllClockSource};
//!
//! let bus = MipiDsi::new(
//!     peripherals.MIPI_DSI,
//!     peripherals.VDMA,
//!     Config {
//!         num_data_lanes: DataLanes::_2,
//!         lane_bit_rate_mbps: 500.0,
//!         phy_clk_src: PhyPllClockSource::Xtal,
//!         force_clock_lane_hs: false,
//!     },
//! )
//! .unwrap();
//! # {after_snippet}
//! ```

pub mod dbi;
pub mod dpi;
pub(crate) mod vdma;

use crate::{
    peripherals::{HP_SYS_CLKRST, MIPI_DSI, MIPI_DSI_BRIDGE, MIPI_DSI_HOST, PMU, VDMA},
    rom,
    system::{GenericPeripheralGuard, Peripheral},
};

// ── Constants ─────────────────────────────────────────────────────────────────

const TIMEOUT_CLOCK_FREQ_MHZ: f32 = 10.0;
const ESCAPE_CLOCK_FREQ_MHZ: f32 = 18.0;

// ── Public types ──────────────────────────────────────────────────────────────

/// Number of MIPI D-PHY data lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataLanes {
    /// 1 data lane.
    _1 = 1,
    /// 2 data lanes.
    _2 = 2,
}

/// PLL reference clock source for the D-PHY.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhyPllClockSource {
    /// Crystal oscillator (40 MHz on ESP32-P4).
    Xtal = 0,
    /// Audio PLL.
    Apll = 1,
    /// CPU PLL.
    Cpll = 2,
    /// System PLL.
    Spll = 3,
    /// MSPI PLL.
    Mpll = 4,
}

impl PhyPllClockSource {
    /// Reference clock frequency in MHz for PLL M/N calculation.
    fn freq_mhz(self) -> f32 {
        match self {
            Self::Xtal => 40.0,
            Self::Apll => 40.0, // APLL default; caller may differ
            Self::Cpll => 480.0,
            Self::Spll => 480.0,
            Self::Mpll => 400.0,
        }
    }
}

/// D-PHY configuration clock source (used for all PHY modes except shutdown).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhyCfgClockSource {
    /// 20 MHz from PLL.
    PllF20m = 0,
    /// Fast RC oscillator (~17.5 MHz).
    RcFast = 1,
    /// 25 MHz from PLL.
    PllF25m = 2,
}

/// Bus-level configuration.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Number of active D-PHY data lanes.
    pub num_data_lanes: DataLanes,
    /// Lane bit rate in Mbps (80–1500).
    pub lane_bit_rate_mbps: f32,
    /// PLL reference clock source.
    pub phy_clk_src: PhyPllClockSource,
    /// Keep the clock lane continuously in HS mode (`true`) or use auto mode
    /// where the DSI host manages HS↔LP transitions (`false`).
    pub force_clock_lane_hs: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            num_data_lanes: DataLanes::_2,
            lane_bit_rate_mbps: 500.0,
            phy_clk_src: PhyPllClockSource::Xtal,
            force_clock_lane_hs: false,
        }
    }
}

/// Configuration error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// Lane bit rate is outside the 80–1500 Mbps range.
    InvalidLaneBitRate,
    /// No valid PLL M/N solution found for the requested bit rate.
    PllNoSolution,
}

// ── MipiDsi bus struct ────────────────────────────────────────────────────────

/// Owns the DSI/VDMA peripherals and powers down the D-PHY on drop.
///
/// Kept separate from [`MipiDsi`] so it can be moved into [`dpi::DsiDpi`]
/// without triggering the "cannot move out of a type that implements `Drop`"
/// restriction on the outer bus struct.
pub(crate) struct DphyGuard<'d> {
    pub(crate) _dsi:        MIPI_DSI<'d>,
    pub(crate) _vdma:       VDMA<'d>,
    pub(crate) _dsi_guard:  GenericPeripheralGuard<{ Peripheral::MipiDsi as u8 }>,
    pub(crate) _vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }>,
}

impl Drop for DphyGuard<'_> {
    fn drop(&mut self) {
        dphy_power_down();
    }
}

/// MIPI DSI bus.
///
/// Holds ownership of both the `MIPI_DSI` and `VDMA` virtual peripherals and
/// keeps their clocks enabled for as long as this value lives.
pub struct MipiDsi<'d> {
    pub(crate) guard: DphyGuard<'d>,
    /// Actual lane bit rate after PLL rounding (MHz).
    #[allow(dead_code)]
    pub(crate) lane_bit_rate_mbps: f32,
    #[allow(dead_code)]
    pub(crate) num_data_lanes: DataLanes,
}

impl<'d> MipiDsi<'d> {
    /// Initialise the MIPI DSI bus.
    ///
    /// Enables APB clock, resets the bridge, powers up the D-PHY LDO,
    /// configures D-PHY PLL, and polls for PLL lock + lane-stopped status
    /// before returning.
    pub fn new(
        dsi:    MIPI_DSI<'d>,
        vdma:   VDMA<'d>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        if config.lane_bit_rate_mbps < 80.0 || config.lane_bit_rate_mbps > 1500.0 {
            return Err(ConfigError::InvalidLaneBitRate);
        }

        dphy_ldo_power_on();

        // Enable APB clock and reset bridge (via GenericPeripheralGuard).
        let dsi_guard:  GenericPeripheralGuard<{ Peripheral::MipiDsi as u8 }> =
            GenericPeripheralGuard::new();
        let vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }> =
            GenericPeripheralGuard::new();

        // PHY configuration clock: always use PLL_F20M (0) as default.
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl02()
            .modify(|_, w| unsafe { w.mipi_dsi_dphy_clk_src_sel().bits(0) });
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_dsi_dphy_cfg_clk_en().set_bit());

        // PHY PLL reference clock.
        HP_SYS_CLKRST::regs().peri_clk_ctrl03().modify(|_, w| unsafe {
            w.mipi_dsi_dphy_pll_refclk_src_sel()
                .bits(config.phy_clk_src as u8)
                .mipi_dsi_dphy_pll_refclk_div_num()
                .bits(0) // div=1 → reg = div-1 = 0
                .mipi_dsi_dphy_pll_refclk_en()
                .set_bit()
        });

        let host = MIPI_DSI_HOST::regs();
        let bridge = MIPI_DSI_BRIDGE::regs();

        // Set data lane count.
        host.phy_if_cfg().modify(|_, w| unsafe {
            w.n_lanes().bits(config.num_data_lanes as u8 - 1)
        });

        // Power on host and PHY.
        host.pwr_up().modify(|_, w| w.shutdownz().set_bit());
        host.phy_rstz().modify(|_, w| w.phy_shutdownz().set_bit());

        // Reset PHY digital section (0→1).
        host.phy_rstz().modify(|_, w| w.phy_rstz().clear_bit());
        host.phy_rstz().modify(|_, w| w.phy_rstz().set_bit());

        // Enable clock lane and force PLL.
        host.phy_rstz()
            .modify(|_, w| w.phy_enableclk().set_bit().phy_forcepll().set_bit());

        // Internal bridge soft-reset.
        bridge.en().modify(|_, w| w.dsi_brig_rst().set_bit());
        bridge.en().modify(|_, w| w.dsi_brig_rst().clear_bit());

        // Configure D-PHY PLL via test interface.
        let (pll_m, pll_n, actual_rate) =
            compute_pll(config.phy_clk_src.freq_mhz(), config.lane_bit_rate_mbps)
                .ok_or(ConfigError::PllNoSolution)?;

        let hs_freq_sel = hs_freq_range(config.lane_bit_rate_mbps);
        phy_write(0x44, hs_freq_sel << 1);
        phy_write(0x19, 0x30); // use external N and M
        phy_write(0x17, pll_n - 1);
        phy_write(0x18, ((pll_m - 1) & 0x1F) as u8);
        phy_write(0x18, 0x80 | (((pll_m - 1) >> 5) & 0x0F) as u8);

        // Poll PLL lock.
        while !host.phy_status().read().phy_lock().bit_is_set() {}

        // Poll lane-stopped.
        let lane_mask: u32 = {
            let mut m = 1 << 2; // clock lane
            if config.num_data_lanes as u8 >= 1 { m |= 1 << 4; }
            if config.num_data_lanes as u8 >= 2 { m |= 1 << 7; }
            m
        };
        while (host.phy_status().read().bits() & lane_mask) != lane_mask {}

        // Enter command mode.
        host.mode_cfg().modify(|_, w| w.cmd_video_mode().set_bit());

        // Clock lane state.
        if config.force_clock_lane_hs {
            host.lpclk_ctrl().modify(|_, w| {
                w.auto_clklane_ctrl().clear_bit().phy_txrequestclkhs().set_bit()
            });
        } else {
            host.lpclk_ctrl().modify(|_, w| {
                w.auto_clklane_ctrl().set_bit().phy_txrequestclkhs().set_bit()
            });
        }

        // HS↔LP switch timings (conservative IDF defaults).
        host.phy_tmr_cfg().modify(|_, w| unsafe {
            w.phy_hs2lp_time().bits(50).phy_lp2hs_time().bits(104)
        });
        host.phy_tmr_lpclk_cfg().modify(|_, w| unsafe {
            w.phy_clkhs2lp_time().bits(46).phy_clklp2hs_time().bits(128)
        });

        // CRC, ECC, EoTp.
        host.pckhdl_cfg().modify(|_, w| {
            w.crc_rx_en().set_bit().ecc_rx_en().set_bit().eotp_tx_en().set_bit()
        });

        // Timeout / escape clock divisors (from byte clock = lane_bit_rate / 8).
        let byte_clock_mhz = actual_rate / 8.0;
        let to_div  = fround_u8(byte_clock_mhz / TIMEOUT_CLOCK_FREQ_MHZ);
        let esc_div = fround_u8(byte_clock_mhz / ESCAPE_CLOCK_FREQ_MHZ);
        host.clkmgr_cfg().modify(|_, w| unsafe {
            w.to_clk_division().bits(to_div).tx_esc_clk_division().bits(esc_div)
        });

        // Disable all timeouts (set to 0).
        host.to_cnt_cfg()
            .write(|w| unsafe { w.bits(0) });
        host.hs_rd_to_cnt()
            .write(|w| unsafe { w.bits(0) });
        host.lp_rd_to_cnt()
            .write(|w| unsafe { w.bits(0) });
        host.hs_wr_to_cnt()
            .write(|w| unsafe { w.bits(0) });
        host.lp_wr_to_cnt()
            .write(|w| unsafe { w.bits(0) });
        host.bta_to_cnt()
            .write(|w| unsafe { w.bits(0) });

        // Max read time and stop-wait time.
        host.phy_tmr_rd_cfg()
            .modify(|_, w| unsafe { w.max_rd_time().bits(6000) });
        host.phy_if_cfg()
            .modify(|_, w| unsafe { w.phy_stop_wait_time().bits(0x3F) });

        Ok(Self {
            guard: DphyGuard { _dsi: dsi, _vdma: vdma, _dsi_guard: dsi_guard, _vdma_guard: vdma_guard },
            lane_bit_rate_mbps: actual_rate,
            num_data_lanes: config.num_data_lanes,
        })
    }

    /// Attach a command-mode (DBI) interface for panel init sequences.
    pub fn dbi(&mut self, virtual_channel: u8) -> dbi::DsiDbi<'_, 'd> {
        dbi::DsiDbi::new(self, virtual_channel)
    }

    /// Enter video-mode (DPI) streaming.
    ///
    /// Consumes `self`; the returned [`DsiDpi`] keeps clocks enabled for the
    /// lifetime of the streaming session.
    ///
    /// `framebuffers` must contain 1–3 slices, each of exactly
    /// `h_active × v_active × bytes_per_pixel` bytes, 64-byte aligned, and
    /// allocated from PSRAM.
    pub fn dpi(
        self,
        config: dpi::DpiConfig,
        framebuffers: &[&'d mut [u8]],
    ) -> Result<dpi::DsiDpi<'d>, ConfigError> {
        dpi::DsiDpi::new(self, config, framebuffers)
    }
}

// ── D-PHY power ───────────────────────────────────────────────────────────────

/// Power up the internal LDO that supplies VDD_MIPI_DPHY at 2.5 V.
///
/// On ESP32-P4 the DPHY power rail (VDD_MIPI_DPHY) is driven by PMU
/// internal LDO channel 3 (VO3), which maps to the `ext_ldo_p0_0p2a`
/// register pair.
///
/// Without eFuse calibration the nominal output formula gives:
///   Vref = 1.0 V  (dref = 9)
///   Vout = Vref × (1 + 0.25 × mul) = 2.5 V  (mul = 6)
pub(crate) fn dphy_ldo_power_on() {
    // force_tieh_sel[7]=1 → SW-owned, xpd[8]=1 → enabled.
    // target0/target1 kept at reset values (0x80 / 0x40).
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0180) });
    // dref[31:28]=9, mul[25:23]=6 → 2.5 V.
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits((9u32 << 28) | (6u32 << 23)) });
    // Allow output to settle before the PHY analog circuits start.
    rom::ets_delay_us(500);
}

/// Shut down the D-PHY and disable its LDO.
///
/// Mirrors `dphy_ldo_power_on` in reverse: reset the PHY digital and
/// analog sections, power off the DSI host, then clear the LDO `xpd` bit
/// so VDD_MIPI_DPHY is no longer driven.
pub(crate) fn dphy_power_down() {
    let host = MIPI_DSI_HOST::regs();
    host.phy_rstz().modify(|_, w| {
        w.phy_rstz().clear_bit()
            .phy_shutdownz().clear_bit()
            .phy_enableclk().clear_bit()
    });
    host.pwr_up().modify(|_, w| w.shutdownz().clear_bit());
    // Restore LDO to reset state (xpd=0, SW ownership released).
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
}

// ── PLL helpers ───────────────────────────────────────────────────────────────

/// PLL frequency range table (start_mbps, end_mbps, hs_freq_range_sel).
const PHY_PLL_RANGES: &[(u16, u16, u8)] = &[
    (80,   89,  0x00), (90,   99,  0x10), (100,  109, 0x20), (110,  129, 0x01),
    (130,  139, 0x11), (140,  149, 0x21), (150,  169, 0x02), (170,  179, 0x12),
    (180,  199, 0x22), (200,  219, 0x03), (220,  239, 0x13), (240,  249, 0x23),
    (250,  269, 0x04), (270,  299, 0x14), (300,  329, 0x05), (330,  359, 0x15),
    (360,  399, 0x25), (400,  449, 0x06), (450,  499, 0x16), (500,  549, 0x07),
    (550,  599, 0x17), (600,  649, 0x08), (650,  699, 0x18), (700,  749, 0x09),
    (750,  799, 0x19), (800,  849, 0x29), (850,  899, 0x39), (900,  949, 0x0A),
    (950,  999, 0x1A), (1000, 1049, 0x2A), (1050, 1099, 0x3A), (1100, 1149, 0x0B),
    (1150, 1199, 0x1B), (1200, 1249, 0x2B), (1250, 1299, 0x3B), (1300, 1349, 0x0C),
    (1350, 1399, 0x1C), (1400, 1449, 0x2C), (1450, 1500, 0x3C),
];

fn hs_freq_range(lane_mbps: f32) -> u8 {
    let mbps = lane_mbps as u16;
    for &(start, end, sel) in PHY_PLL_RANGES {
        if mbps >= start && mbps <= end {
            return sel;
        }
    }
    0x00
}

/// Compute PLL M and N divisors.
///
/// Constraint: 5 MHz ≤ f_ref/N ≤ 40 MHz, M must be even.
/// Returns `(M, N, actual_lane_rate_mbps)` or `None` if no solution found.
fn compute_pll(ref_freq_mhz: f32, target_mbps: f32) -> Option<(u16, u8, f32)> {
    let min_n = ((ref_freq_mhz / 40.0) as u8).max(1);
    let max_n = (ref_freq_mhz / 5.0) as u8;
    let mut best_m: u16 = 0;
    let mut best_n: u8 = 0;
    let mut best_delta = f32::INFINITY;

    for n in min_n..=max_n {
        let m_raw = (target_mbps * n as f32 / ref_freq_mhz) as u16;
        // M must be even; try m_raw rounded down to even.
        let m = m_raw & !1;
        if m == 0 { continue; }
        let actual = ref_freq_mhz * m as f32 / n as f32;
        let delta = (target_mbps - actual).abs();
        if delta < best_delta {
            best_delta = delta;
            best_m = m;
            best_n = n;
            if best_delta < 0.01 { break; }
        }
    }

    if best_m == 0 || best_n == 0 { return None; }
    let actual = ref_freq_mhz * best_m as f32 / best_n as f32;
    Some((best_m, best_n, actual))
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Round a positive f32 to the nearest u8 without using `f32::round()`.
#[inline]
fn fround_u8(x: f32) -> u8 {
    (x + 0.5) as u8
}

// ── D-PHY test interface bit-bang ─────────────────────────────────────────────

fn phy_write(addr: u8, val: u8) {
    let h = MIPI_DSI_HOST::regs();
    h.phy_tst_ctrl0()
        .write(|w| w.phy_testclk().clear_bit().phy_testclr().clear_bit());
    // Write address with TESTEN=1.
    h.phy_tst_ctrl1()
        .write(|w| unsafe { w.phy_testen().set_bit().phy_testdin().bits(addr) });
    h.phy_tst_ctrl0()
        .write(|w| w.phy_testclk().set_bit().phy_testclr().clear_bit());
    h.phy_tst_ctrl0()
        .write(|w| w.phy_testclk().clear_bit().phy_testclr().clear_bit());
    // Write value with TESTEN=0.
    h.phy_tst_ctrl1()
        .write(|w| unsafe { w.phy_testen().clear_bit().phy_testdin().bits(val) });
    h.phy_tst_ctrl0()
        .write(|w| w.phy_testclk().set_bit().phy_testclr().clear_bit());
    h.phy_tst_ctrl0()
        .write(|w| w.phy_testclk().clear_bit().phy_testclr().clear_bit());
}

#![cfg_attr(docsrs, procmacros::doc_replace)]
//! MIPI DSI bus driver (ESP32-P4).
//!
//! Wraps `MIPI_DSI_HOST` and `MIPI_DSI_BRIDGE` register blocks behind two
//! virtual-peripheral ownership tokens (`MIPI_DSI` + one `VDMA_CH*` channel).
//!
//! # Example
//!
//! ```rust,no_run
//! # {before_snippet}
//! use esp_hal::{
//!     clock::ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
//!     mipi_dsi::{Config, DataLanes, MipiDsi},
//! };
//!
//! let bus = MipiDsi::new(
//!     peripherals.MIPI_DSI,
//!     peripherals.VDMA_CH0,
//!     Config::default()
//!         .with_num_data_lanes(DataLanes::_2)
//!         .with_lane_bit_rate_mbps(500.0)
//!         .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
//!             MipiDsiPhyPllRefclkSclk::Xtal, // XTAL as reference
//!             0,                             // divider = 1 (div_num register = 0)
//!         ))
//!         .with_force_clock_lane_hs(false),
//! )
//! .unwrap();
//! # {after_snippet}
//! ```

pub mod dbi;
pub mod dpi;
pub(crate) mod vdma;

use core::marker::PhantomData;

use dpi::DsiDpi;
use procmacros::BuilderLite;
pub use vdma::VdmaDmaChannel;

use crate::{
    peripherals::{MIPI_DSI, MIPI_DSI_BRIDGE, MIPI_DSI_HOST, PMU},
    rom,
    soc::clocks::{
        ClockTree,
        MipiDsiInstance,
        MipiDsiPhyCfgClkConfig,
        MipiDsiPhyPllRefclkConfig,
        MipiDsiPhyPllRefclkSclk,
    },
    system::{GenericPeripheralGuard, Peripheral},
};

// ── Constants ─────────────────────────────────────────────────────────────────

const TIMEOUT_CLOCK_FREQ_MHZ: f32 = 10.0;
const ESCAPE_CLOCK_FREQ_MHZ: f32 = 18.0;

// ── Public types ──────────────────────────────────────────────────────────────

/// Number of MIPI D-PHY data lanes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataLanes {
    /// 1 data lane.
    _1 = 1,

    /// 2 data lanes.
    _2 = 2,
}

/// Bus-level configuration.
#[derive(Debug, Clone, Copy, PartialEq, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Number of active D-PHY data lanes.
    num_data_lanes: DataLanes,

    /// Lane bit rate in Mbps (80–1500).
    lane_bit_rate_mbps: f32,

    /// PLL reference clock source for the D-PHY.
    ///
    /// APLL is not yet modelled in the clock tree and cannot be selected here.
    phy_pll_refclk: MipiDsiPhyPllRefclkConfig,

    /// Keep the clock lane continuously in HS mode (`true`) or use auto mode
    /// where the DSI host manages HS↔LP transitions (`false`).
    force_clock_lane_hs: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            num_data_lanes: DataLanes::_2,
            lane_bit_rate_mbps: 500.0,
            phy_pll_refclk: MipiDsiPhyPllRefclkConfig::new(MipiDsiPhyPllRefclkSclk::Xtal, 0),
            force_clock_lane_hs: false,
        }
    }
}

/// Configuration error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Lane bit rate is outside the 80–1500 Mbps range.
    InvalidLaneBitRate,

    /// No valid PLL M/N solution found for the requested bit rate.
    PllNoSolution,
}

// ── MipiDsi bus struct ────────────────────────────────────────────────────────

/// Owns the DSI peripheral and one VDMA channel; powers down the D-PHY on drop.
pub(crate) struct DphyGuard<'d> {
    pub(crate) _dsi: MIPI_DSI<'d>,
    /// The hardware channel index (0–3) of the VDMA channel in use.
    pub(crate) vdma_channel_id: u8,
    pub(crate) _dsi_guard: GenericPeripheralGuard<{ Peripheral::MipiDsi as u8 }>,
    pub(crate) _vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }>,
    pub(crate) _phantom: PhantomData<&'d mut ()>,
}

impl Drop for DphyGuard<'_> {
    fn drop(&mut self) {
        dphy_power_down();
        ClockTree::with(|clocks| {
            MipiDsiInstance::MipiDsi.release_phy_pll_refclk(clocks);
            MipiDsiInstance::MipiDsi.release_phy_cfg_clk(clocks);
        });
    }
}

/// MIPI DSI bus.
///
/// Holds ownership of the `MIPI_DSI` virtual peripheral and one VDMA channel
/// singleton, and keeps their clocks enabled for as long as this value lives.
pub struct MipiDsi<'d> {
    pub(crate) guard: DphyGuard<'d>,
    /// Actual lane bit rate after PLL rounding (MHz).
    pub(crate) lane_bit_rate_mbps: f32,
}

impl<'d> MipiDsi<'d> {
    /// Initialise the MIPI DSI bus.
    ///
    /// Enables APB clock, resets the bridge, powers up the D-PHY LDO,
    /// configures D-PHY PLL, and polls for PLL lock + lane-stopped status
    /// before returning.
    ///
    /// `vdma_channel` is one of the four `VDMA_CH*` peripheral singletons;
    /// it is consumed to ensure exclusive access to that DMA channel for the
    /// lifetime of the bus.
    pub fn new(
        dsi: MIPI_DSI<'d>,
        vdma_channel: impl VdmaDmaChannel + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        if config.lane_bit_rate_mbps < 80.0 || config.lane_bit_rate_mbps > 1500.0 {
            return Err(ConfigError::InvalidLaneBitRate);
        }

        let vdma_channel_id = vdma_channel.channel_id();

        dphy_ldo_power_on();

        // Enable APB clock and reset bridge (via GenericPeripheralGuard).
        let dsi_guard: GenericPeripheralGuard<{ Peripheral::MipiDsi as u8 }> =
            GenericPeripheralGuard::new();
        let vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }> =
            GenericPeripheralGuard::new();

        // Configure and enable PHY clocks through the clock tree so upstream
        // PLLs are reference-counted and the register writes are centralised in
        // the clock-tree implementation functions in clocks.rs.
        let ref_freq_hz = ClockTree::with(|clocks| {
            // D-PHY configuration clock: always PLL_F20M.
            MipiDsiInstance::MipiDsi.configure_phy_cfg_clk(clocks, MipiDsiPhyCfgClkConfig::PllF20m);
            MipiDsiInstance::MipiDsi.request_phy_cfg_clk(clocks);

            // D-PHY PLL reference clock (user-selected source).
            MipiDsiInstance::MipiDsi.configure_phy_pll_refclk(clocks, config.phy_pll_refclk);
            MipiDsiInstance::MipiDsi.request_phy_pll_refclk(clocks);

            MipiDsiInstance::phy_pll_refclk_config_frequency(clocks, config.phy_pll_refclk)
        });
        let ref_freq_mhz = ref_freq_hz as f32 / 1_000_000.0;

        let dphy_guard = DphyGuard {
            _dsi: dsi,
            vdma_channel_id,
            _dsi_guard: dsi_guard,
            _vdma_guard: vdma_guard,
            _phantom: PhantomData,
        };

        let host = MIPI_DSI_HOST::regs();
        let bridge = MIPI_DSI_BRIDGE::regs();

        // Set data lane count.
        host.phy_if_cfg()
            .modify(|_, w| unsafe { w.n_lanes().bits(config.num_data_lanes as u8 - 1) });

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
        let (pll_m, pll_n, actual_rate) = compute_pll(ref_freq_mhz, config.lane_bit_rate_mbps)
            .ok_or(ConfigError::PllNoSolution)?;

        let hs_freq_sel = hs_freq_range(actual_rate);
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
            if config.num_data_lanes as u8 >= 1 {
                m |= 1 << 4;
            }
            if config.num_data_lanes as u8 >= 2 {
                m |= 1 << 7;
            }
            m
        };
        while (host.phy_status().read().bits() & lane_mask) != lane_mask {}

        // Enter command mode.
        host.mode_cfg().modify(|_, w| w.cmd_video_mode().set_bit());

        // Clock lane state.
        if config.force_clock_lane_hs {
            host.lpclk_ctrl().modify(|_, w| {
                w.auto_clklane_ctrl().clear_bit();
                w.phy_txrequestclkhs().set_bit()
            });
        } else {
            host.lpclk_ctrl().modify(|_, w| {
                w.auto_clklane_ctrl().set_bit();
                w.phy_txrequestclkhs().set_bit()
            });
        }

        // HS↔LP switch timings (conservative IDF defaults).
        host.phy_tmr_cfg().modify(|_, w| unsafe {
            w.phy_hs2lp_time().bits(50);
            w.phy_lp2hs_time().bits(104)
        });
        host.phy_tmr_lpclk_cfg().modify(|_, w| unsafe {
            w.phy_clkhs2lp_time().bits(46);
            w.phy_clklp2hs_time().bits(128)
        });

        // CRC, ECC, EoTp.
        host.pckhdl_cfg().modify(|_, w| {
            w.crc_rx_en().set_bit();
            w.ecc_rx_en().set_bit();
            w.eotp_tx_en().set_bit()
        });

        // Timeout / escape clock divisors (from byte clock = lane_bit_rate / 8).
        let byte_clock_mhz = actual_rate / 8.0;
        let to_div = fround_u8(byte_clock_mhz / TIMEOUT_CLOCK_FREQ_MHZ);
        let esc_div = fround_u8(byte_clock_mhz / ESCAPE_CLOCK_FREQ_MHZ);
        host.clkmgr_cfg().modify(|_, w| unsafe {
            w.to_clk_division()
                .bits(to_div)
                .tx_esc_clk_division()
                .bits(esc_div)
        });

        // Disable all timeouts (set to 0).
        host.to_cnt_cfg().write(|w| unsafe { w.bits(0) });
        host.hs_rd_to_cnt().write(|w| unsafe { w.bits(0) });
        host.lp_rd_to_cnt().write(|w| unsafe { w.bits(0) });
        host.hs_wr_to_cnt().write(|w| unsafe { w.bits(0) });
        host.lp_wr_to_cnt().write(|w| unsafe { w.bits(0) });
        host.bta_to_cnt().write(|w| unsafe { w.bits(0) });

        // Max read time and stop-wait time.
        host.phy_tmr_rd_cfg()
            .modify(|_, w| unsafe { w.max_rd_time().bits(6000) });
        host.phy_if_cfg()
            .modify(|_, w| unsafe { w.phy_stop_wait_time().bits(0x3F) });

        Ok(Self {
            guard: dphy_guard,
            lane_bit_rate_mbps: actual_rate,
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
    ) -> Result<DsiDpi<'d>, ConfigError> {
        DsiDpi::new(self, config, framebuffers)
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
        w.phy_rstz().clear_bit();
        w.phy_shutdownz().clear_bit();
        w.phy_enableclk().clear_bit()
    });
    host.pwr_up().modify(|_, w| w.shutdownz().clear_bit());
    // Restore LDO to reset state (xpd=0, SW ownership released).
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(0x4020_0000) });
}

// ── PLL helpers ───────────────────────────────────────────────────────────────

/// PLL frequency range table: half-open Mbps range → `hs_freq_range_sel`.
const PHY_PLL_RANGES: &[(core::ops::Range<u16>, u8)] = &[
    (80..90, 0x00),
    (90..100, 0x10),
    (100..110, 0x20),
    (110..130, 0x01),
    (130..140, 0x11),
    (140..150, 0x21),
    (150..170, 0x02),
    (170..180, 0x12),
    (180..200, 0x22),
    (200..220, 0x03),
    (220..240, 0x13),
    (240..250, 0x23),
    (250..270, 0x04),
    (270..300, 0x14),
    (300..330, 0x05),
    (330..360, 0x15),
    (360..400, 0x25),
    (400..450, 0x06),
    (450..500, 0x16),
    (500..550, 0x07),
    (550..600, 0x17),
    (600..650, 0x08),
    (650..700, 0x18),
    (700..750, 0x09),
    (750..800, 0x19),
    (800..850, 0x29),
    (850..900, 0x39),
    (900..950, 0x0A),
    (950..1000, 0x1A),
    (1000..1050, 0x2A),
    (1050..1100, 0x3A),
    (1100..1150, 0x0B),
    (1150..1200, 0x1B),
    (1200..1250, 0x2B),
    (1250..1300, 0x3B),
    (1300..1350, 0x0C),
    (1350..1400, 0x1C),
    (1400..1450, 0x2C),
    (1450..1501, 0x3C),
];

fn hs_freq_range(lane_mbps: f32) -> u8 {
    let mbps = lane_mbps as u16;
    PHY_PLL_RANGES
        .iter()
        .find(|(range, _)| range.contains(&mbps))
        .map_or(0x00, |&(_, sel)| sel)
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
        if m == 0 {
            continue;
        }
        let actual = ref_freq_mhz * m as f32 / n as f32;
        let delta = (target_mbps - actual).abs();
        if delta < best_delta {
            best_delta = delta;
            best_m = m;
            best_n = n;
            if best_delta < 0.01 {
                break;
            }
        }
    }

    if best_m == 0 || best_n == 0 {
        return None;
    }
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
    h.phy_tst_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().clear_bit()
    });
    // Write address with TESTEN=1.
    h.phy_tst_ctrl1().write(|w| unsafe {
        w.phy_testen().set_bit();
        w.phy_testdin().bits(addr)
    });
    h.phy_tst_ctrl0().write(|w| {
        w.phy_testclk().set_bit();
        w.phy_testclr().clear_bit()
    });
    h.phy_tst_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().clear_bit()
    });
    // Write value with TESTEN=0.
    h.phy_tst_ctrl1().write(|w| unsafe {
        w.phy_testen().clear_bit();
        w.phy_testdin().bits(val)
    });
    h.phy_tst_ctrl0().write(|w| {
        w.phy_testclk().set_bit();
        w.phy_testclr().clear_bit()
    });
    h.phy_tst_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().clear_bit()
    });
}

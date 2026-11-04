#![cfg_attr(docsrs, procmacros::doc_replace)]
//! MIPI CSI-2 receiver controller (ESP32-P4).
//!
//! This is a minimal bring-up milestone. It only powers up and initializes the
//! CSI HOST + CSI BRIDGE controller state into an idle, disabled state.
//! Capture/DMA is intentionally not configured yet.
//!
//! # Power requirements
//!
//! `VDD_MIPI_DPHY` must already be powered and stable before calling
//! [`MipiCsi::new`]. This module does not currently own or control the PMU VO3
//! rail that supplies the D-PHY domain.
//!
//! # Example
//!
//! ```rust,no_run
//! # {before_snippet}
//! use esp_hal::mipi_csi::{Config, DataLanes, MipiCsi};
//!
//! let csi = MipiCsi::new(
//!     peripherals.MIPI_CSI,
//!     Config::default()
//!         .with_num_data_lanes(DataLanes::_2)
//!         .with_lane_bit_rate_mbps(500),
//! )
//! .unwrap();
//! # {after_snippet}
//! ```

use procmacros::BuilderLite;

use crate::{
    peripherals::{HP_SYS_CLKRST, MIPI_CSI, MIPI_CSI_BRIDGE, MIPI_CSI_HOST},
    system::{GenericPeripheralGuard, Peripheral},
};

const MIN_LANE_RATE_MBPS: u16 = 90;
const MAX_LANE_RATE_MBPS: u16 = 1500;

// IDF: MIPI_CSI_PHY_CLK_SRC_PLL_F20M
const PHY_CFG_CLK_SRC_PLL_F20M: u8 = 0;

/// Number of MIPI D-PHY data lanes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataLanes {
    /// 1 data lane.
    _1 = 1,

    /// 2 data lanes.
    _2 = 2,
}

/// MIPI CSI controller configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Number of active D-PHY data lanes.
    num_data_lanes: DataLanes,

    /// Target lane bit rate in Mbps.
    lane_bit_rate_mbps: u16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            num_data_lanes: DataLanes::_2,
            lane_bit_rate_mbps: 500,
        }
    }
}

/// Configuration error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Lane bit rate is outside the 90–1500 Mbps range.
    InvalidLaneBitRate,
}

/// D-PHY configuration clock source.
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhyCfgClockSource {
    /// PLL_F20M source.
    PllF20m,
    /// RC_FAST source.
    RcFast,
    /// PLL_F25M source.
    PllF25m,
    /// An unknown raw value in the source-select field.
    Unknown(u8),
}

/// Readback of CSI initialization-related configuration registers.
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ConfigReadback {
    /// CSI HOST bus clock gate state (`soc_clk_ctrl1.csi_host_sys_clk_en`).
    pub host_bus_clock_enabled: bool,
    /// CSI BRIDGE bus clock gate state (`soc_clk_ctrl1.csi_brg_sys_clk_en`).
    pub bridge_bus_clock_enabled: bool,
    /// D-PHY configuration clock gate state (`peri_clk_ctrl03.mipi_csi_dphy_cfg_clk_en`).
    pub phy_cfg_clock_enabled: bool,
    /// D-PHY configuration clock source (`peri_clk_ctrl03.mipi_csi_dphy_clk_src_sel`).
    pub phy_cfg_clock_source: PhyCfgClockSource,
    /// Bridge-controlled CSI host clock-lane module state (`host_ctrl.csi_enableclk`).
    pub bridge_internal_clock_enabled: bool,
    /// CSI HOST reset state (`hp_rst_en0.rst_en_csi_host`).
    pub host_bus_reset_asserted: bool,
    /// CSI BRIDGE reset state (`hp_rst_en0.rst_en_csi_brg`).
    pub bridge_bus_reset_asserted: bool,
    /// CSI BRIDGE local reset state (`csi_en.csi_brig_rst`).
    pub bridge_soft_reset_asserted: bool,
    /// D-PHY shutdown state (`phy_shutdownz.phy_shutdownz`, active-low).
    pub phy_shutdown_asserted: bool,
    /// D-PHY reset state (`dphy_rstz.dphy_rstz`, active-low).
    pub dphy_reset_asserted: bool,
    /// CSI-2 controller reset state (`csi2_resetn.csi2_resetn`, active-low).
    pub csi2_controller_reset_asserted: bool,
    /// Configured active data-lane count (`n_lanes + 1`).
    pub configured_data_lanes: u8,
    /// CSI virtual-channel extension mode.
    pub virtual_channel_extension_enabled: bool,
    /// CSI host descrambling state.
    pub scrambling_enabled: bool,
    /// CSI bridge enable state (`csi_en.csi_brig_en`).
    pub bridge_enabled: bool,
}

/// CSI host error interrupt summary status.
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HostErrorStatus {
    /// Any PHY fatal status.
    pub phy_fatal: bool,
    /// Any packet fatal status.
    pub packet_fatal: bool,
    /// Any frame-boundary fatal status.
    pub boundary_frame_fatal: bool,
    /// Any frame-sequence fatal status.
    pub sequence_frame_fatal: bool,
    /// Any frame CRC fatal status.
    pub crc_frame_fatal: bool,
    /// Any payload CRC fatal status.
    pub payload_crc_fatal: bool,
    /// Any unsupported data-id status.
    pub data_id_error: bool,
    /// Any corrected header ECC status.
    pub ecc_corrected: bool,
    /// Any PHY non-fatal status.
    pub phy_error: bool,
    /// Raw `int_st_main` value for debugging.
    pub raw_main_status: u32,
}

#[cfg(feature = "unstable")]
impl HostErrorStatus {
    /// Returns true when any host error/status bit is set.
    pub const fn any_error(&self) -> bool {
        self.phy_fatal
            || self.packet_fatal
            || self.boundary_frame_fatal
            || self.sequence_frame_fatal
            || self.crc_frame_fatal
            || self.payload_crc_fatal
            || self.data_id_error
            || self.ecc_corrected
            || self.phy_error
    }
}

/// CSI bridge event summary status.
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BridgeEventStatus {
    /// Configured `vadr_num` greater-than event.
    pub vadr_num_gt: bool,
    /// Configured `vadr_num` less-than event.
    pub vadr_num_lt: bool,
    /// Incomplete-frame discard event.
    pub discard: bool,
    /// Buffer overrun event.
    pub csi_buf_overrun: bool,
    /// Async FIFO overflow event.
    pub csi_async_fifo_ovf: bool,
    /// DMA configuration update-complete event.
    pub dma_cfg_has_updated: bool,
    /// Raw `int_st` value for debugging.
    pub raw_status: u32,
}

#[cfg(feature = "unstable")]
impl BridgeEventStatus {
    /// Returns true when any bridge event/status bit is set.
    pub const fn any_event(&self) -> bool {
        self.vadr_num_gt
            || self.vadr_num_lt
            || self.discard
            || self.csi_buf_overrun
            || self.csi_async_fifo_ovf
            || self.dma_cfg_has_updated
    }
}

/// Runtime hardware status that is not just static configuration readback.
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HardwareStatus {
    /// CSI host error summary.
    pub host_errors: HostErrorStatus,
    /// CSI bridge event summary.
    pub bridge_events: BridgeEventStatus,
}

/// Full initialization diagnostics snapshot.
#[cfg(feature = "unstable")]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InitDiagnostics {
    /// Configuration register readback.
    pub config: ConfigReadback,
    /// Runtime status/event readback.
    pub status: HardwareStatus,
}

/// MIPI CSI controller handle.
///
/// Owns the `MIPI_CSI` virtual peripheral singleton and keeps HOST/BRIDGE
/// clocks enabled for as long as this value lives.
pub struct MipiCsi<'d> {
    _csi: MIPI_CSI<'d>,
    _guard: GenericPeripheralGuard<{ Peripheral::MipiCsi as u8 }>,
}

impl<'d> MipiCsi<'d> {
    /// Initializes MIPI CSI HOST and BRIDGE into an idle state.
    ///
    /// This validates lane rate, enables and resets the peripheral via
    /// `GenericPeripheralGuard`, then applies the minimal verified D-PHY/HOST
    /// sequence for ESP32-P4.
    ///
    /// `VDD_MIPI_DPHY` must already be powered and stable before calling this
    /// constructor.
    pub fn new(csi: MIPI_CSI<'d>, config: Config) -> Result<Self, ConfigError> {
        if !(MIN_LANE_RATE_MBPS..=MAX_LANE_RATE_MBPS).contains(&config.lane_bit_rate_mbps) {
            return Err(ConfigError::InvalidLaneBitRate);
        }

        let this = Self {
            _csi: csi,
            _guard: GenericPeripheralGuard::new(),
        };

        this.init_phy_cfg_clock();
        this.init_host(config);
        this.init_bridge();

        Ok(this)
    }

    /// Returns a readback snapshot of initialization-related CSI state.
    ///
    /// Register readback is useful to verify software configuration, but it
    /// does not prove a working MIPI CSI physical link.
    #[cfg(feature = "unstable")]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    pub fn init_diagnostics(&self) -> InitDiagnostics {
        let host = MIPI_CSI_HOST::regs();
        let bridge = MIPI_CSI_BRIDGE::regs();
        let clkrst = HP_SYS_CLKRST::regs();

        let host_clk = clkrst.soc_clk_ctrl1().read();
        let rst = clkrst.hp_rst_en0().read();
        let phy_cfg_clk = clkrst.peri_clk_ctrl03().read();
        let bridge_csi_en = bridge.csi_en().read();
        let host_int_main = host.int_st_main().read();
        let bridge_int = bridge.int_st().read();

        let phy_cfg_clock_source = match phy_cfg_clk.mipi_csi_dphy_clk_src_sel().bits() {
            0 => PhyCfgClockSource::PllF20m,
            1 => PhyCfgClockSource::RcFast,
            2 => PhyCfgClockSource::PllF25m,
            other => PhyCfgClockSource::Unknown(other),
        };

        InitDiagnostics {
            config: ConfigReadback {
                host_bus_clock_enabled: host_clk.csi_host_sys_clk_en().bit_is_set(),
                bridge_bus_clock_enabled: host_clk.csi_brg_sys_clk_en().bit_is_set(),
                phy_cfg_clock_enabled: phy_cfg_clk.mipi_csi_dphy_cfg_clk_en().bit_is_set(),
                phy_cfg_clock_source,
                bridge_internal_clock_enabled: bridge
                    .host_ctrl()
                    .read()
                    .csi_enableclk()
                    .bit_is_set(),
                host_bus_reset_asserted: rst.rst_en_csi_host().bit_is_set(),
                bridge_bus_reset_asserted: rst.rst_en_csi_brg().bit_is_set(),
                bridge_soft_reset_asserted: bridge_csi_en.csi_brig_rst().bit_is_set(),
                phy_shutdown_asserted: host.phy_shutdownz().read().phy_shutdownz().bit_is_clear(),
                dphy_reset_asserted: host.dphy_rstz().read().dphy_rstz().bit_is_clear(),
                csi2_controller_reset_asserted: host
                    .csi2_resetn()
                    .read()
                    .csi2_resetn()
                    .bit_is_clear(),
                configured_data_lanes: host.n_lanes().read().n_lanes().bits() + 1,
                virtual_channel_extension_enabled: host.vc_extension().read().vcx().bit_is_clear(),
                scrambling_enabled: host.scrambling().read().scramble_enable().bit_is_set(),
                bridge_enabled: bridge_csi_en.csi_brig_en().bit_is_set(),
            },
            status: HardwareStatus {
                host_errors: HostErrorStatus {
                    phy_fatal: host_int_main.st_status_int_phy_fatal().bit_is_set(),
                    packet_fatal: host_int_main.st_status_int_pkt_fatal().bit_is_set(),
                    boundary_frame_fatal: host_int_main
                        .st_status_int_bndry_frame_fatal()
                        .bit_is_set(),
                    sequence_frame_fatal: host_int_main
                        .st_status_int_seq_frame_fatal()
                        .bit_is_set(),
                    crc_frame_fatal: host_int_main.st_status_int_crc_frame_fatal().bit_is_set(),
                    payload_crc_fatal: host_int_main.st_status_int_pld_crc_fatal().bit_is_set(),
                    data_id_error: host_int_main.st_status_int_data_id().bit_is_set(),
                    ecc_corrected: host_int_main.st_status_int_ecc_corrected().bit_is_set(),
                    phy_error: host_int_main.st_status_int_phy().bit_is_set(),
                    raw_main_status: host_int_main.bits(),
                },
                bridge_events: BridgeEventStatus {
                    vadr_num_gt: bridge_int.vadr_num_gt().bit_is_set(),
                    vadr_num_lt: bridge_int.vadr_num_lt().bit_is_set(),
                    discard: bridge_int.discard().bit_is_set(),
                    csi_buf_overrun: bridge_int.csi_buf_overrun().bit_is_set(),
                    csi_async_fifo_ovf: bridge_int.csi_async_fifo_ovf().bit_is_set(),
                    dma_cfg_has_updated: bridge_int.dma_cfg_has_updated().bit_is_set(),
                    raw_status: bridge_int.bits(),
                },
            },
        }
    }

    fn init_phy_cfg_clock(&self) {
        // Match IDF CSI LL default: select PLL_F20M for CSI D-PHY cfg clock,
        // then pulse cfg-clk gate low -> high.
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| unsafe { w.mipi_csi_dphy_clk_src_sel().bits(PHY_CFG_CLK_SRC_PLL_F20M) });
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_csi_dphy_cfg_clk_en().clear_bit());
        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_csi_dphy_cfg_clk_en().set_bit());
    }

    fn init_host(&self, config: Config) {
        let host = MIPI_CSI_HOST::regs();

        // Assert shutdown + resets before touching PHY configuration.
        host.phy_shutdownz()
            .modify(|_, w| w.phy_shutdownz().clear_bit());
        host.dphy_rstz().modify(|_, w| w.dphy_rstz().clear_bit());
        host.csi2_resetn()
            .modify(|_, w| w.csi2_resetn().clear_bit());

        phy_test_reset();
        phy_write(0x44, hs_frequency_range(config.lane_bit_rate_mbps) << 1);

        // Release shutdown + resets in documented order.
        host.phy_shutdownz()
            .modify(|_, w| w.phy_shutdownz().set_bit());
        host.dphy_rstz().modify(|_, w| w.dphy_rstz().set_bit());
        host.csi2_resetn().modify(|_, w| w.csi2_resetn().set_bit());

        // Program lane count, and keep extension/scrambling features disabled.
        host.n_lanes()
            .modify(|_, w| unsafe { w.n_lanes().bits(config.num_data_lanes as u8 - 1) });
        host.vc_extension().modify(|_, w| w.vcx().set_bit());
        host.scrambling()
            .modify(|_, w| w.scramble_enable().clear_bit());
    }

    fn init_bridge(&self) {
        let bridge = MIPI_CSI_BRIDGE::regs();

        bridge
            .host_ctrl()
            .modify(|_, w| w.csi_enableclk().set_bit());

        // Keep the bridge in reset while ensuring `csi_brig_en` stays cleared.
        bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_rst().set_bit().csi_brig_en().clear_bit());
        bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_rst().clear_bit().csi_brig_en().clear_bit());
        bridge.int_ena().reset();
    }

    fn shutdown(&self) {
        let host = MIPI_CSI_HOST::regs();
        let bridge = MIPI_CSI_BRIDGE::regs();

        bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_en().clear_bit().csi_brig_rst().set_bit());
        bridge
            .host_ctrl()
            .modify(|_, w| w.csi_enableclk().clear_bit());

        host.phy_shutdownz()
            .modify(|_, w| w.phy_shutdownz().clear_bit());
        host.dphy_rstz().modify(|_, w| w.dphy_rstz().clear_bit());
        host.csi2_resetn()
            .modify(|_, w| w.csi2_resetn().clear_bit());

        HP_SYS_CLKRST::regs()
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_csi_dphy_cfg_clk_en().clear_bit());
    }
}

impl Drop for MipiCsi<'_> {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn hs_frequency_range(lane_rate_mbps: u16) -> u8 {
    match lane_rate_mbps {
        90..=99 => 0x00,
        100..=109 => 0x10,
        110..=129 => 0x20,
        130..=139 => 0x01,
        140..=149 => 0x11,
        150..=169 => 0x21,
        170..=179 => 0x02,
        180..=199 => 0x12,
        200..=219 => 0x22,
        220..=239 => 0x03,
        240..=249 => 0x13,
        250..=269 => 0x23,
        270..=299 => 0x04,
        300..=329 => 0x14,
        330..=359 => 0x05,
        360..=399 => 0x15,
        400..=449 => 0x25,
        450..=499 => 0x06,
        500..=549 => 0x16,
        550..=599 => 0x07,
        600..=649 => 0x17,
        650..=699 => 0x08,
        700..=749 => 0x18,
        750..=799 => 0x09,
        800..=849 => 0x19,
        850..=899 => 0x29,
        900..=949 => 0x39,
        950..=999 => 0x0A,
        1000..=1049 => 0x1A,
        1050..=1099 => 0x2A,
        1100..=1149 => 0x3A,
        1150..=1199 => 0x0B,
        1200..=1249 => 0x1B,
        1250..=1299 => 0x2B,
        1300..=1349 => 0x3B,
        1350..=1399 => 0x0C,
        1400..=1449 => 0x1C,
        1450..=1500 => 0x2C,
        _ => 0x00,
    }
}

fn phy_test_reset() {
    let host = MIPI_CSI_HOST::regs();

    host.phy_test_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().set_bit()
    });
    host.phy_test_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().clear_bit()
    });
    host.phy_test_ctrl1().write(|w| unsafe {
        w.phy_testen().clear_bit();
        w.phy_testdin().bits(0)
    });
}

fn phy_write(address: u8, value: u8) {
    let host = MIPI_CSI_HOST::regs();

    // Write address with TESTEN=1.
    host.phy_test_ctrl1().write(|w| unsafe {
        w.phy_testen().set_bit();
        w.phy_testdin().bits(address)
    });
    host.phy_test_ctrl0().write(|w| {
        w.phy_testclk().set_bit();
        w.phy_testclr().clear_bit()
    });
    host.phy_test_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().clear_bit()
    });

    // Write value with TESTEN=0.
    host.phy_test_ctrl1().write(|w| unsafe {
        w.phy_testen().clear_bit();
        w.phy_testdin().bits(value)
    });
    host.phy_test_ctrl0().write(|w| {
        w.phy_testclk().set_bit();
        w.phy_testclr().clear_bit()
    });
    host.phy_test_ctrl0().write(|w| {
        w.phy_testclk().clear_bit();
        w.phy_testclr().clear_bit()
    });
}

#![cfg_attr(docsrs, procmacros::doc_replace)]
//! MIPI CSI-2 receiver controller (ESP32-P4).
//!
//! Configures CSI HOST + CSI BRIDGE and supports single-frame blocking
//! peripheral-to-memory capture through the shared DW-GDMA VDMA controller.
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
//! use esp_hal::{
//!     mipi_csi::{CaptureConfig, CapturePixelFormat, Config, DataLanes, MipiCsi},
//!     time::Duration,
//! };
//!
//! let mut capture = MipiCsi::new(
//!     peripherals.MIPI_CSI,
//!     Config::default()
//!         .with_num_data_lanes(DataLanes::_2)
//!         .with_lane_bit_rate_mbps(288),
//! )?
//! .with_dma(
//!     peripherals.VDMA_CH1,
//!     CaptureConfig::default()
//!         .with_frame_width(1024)
//!         .with_frame_height(600)
//!         .with_pixel_format(CapturePixelFormat::Raw8)
//!         .with_timeout(Duration::from_millis(100)),
//! )?;
//!
//! let mut frame = [0u8; 1024 * 600];
//! let _result = capture.receive(&mut frame)?;
//! # {after_snippet}
//! ```

use core::{
    fmt,
    sync::atomic::{AtomicU8, AtomicU32, Ordering},
};

use procmacros::BuilderLite;

use crate::{
    dma::{DmaBufError, aligned::DmaAlignedMut},
    mipi_dw_gdma::{
        AddressIncrementMode,
        BlockTransferConfig,
        BlockTransferEndpointConfig,
        ChannelConfig,
        DwGdmaChannel,
        HandshakeSelect,
        LlPointerMasterSelect,
        MAX_BLOCK_TRANSFER_SIZE_UNITS,
        asserted_interrupt_sources,
        enable_controller,
        enabled_channels,
        register_channel_handler,
        unregister_channel_handler,
    },
    peripherals::{HP_SYS_CLKRST, MIPI_CSI, MIPI_CSI_BRIDGE, MIPI_CSI_HOST},
    private::Sealed,
    system::{GenericPeripheralGuard, Peripheral},
    time::{Duration, Instant},
};

const MIN_LANE_RATE_MBPS: u16 = 90;
const MAX_LANE_RATE_MBPS: u16 = 1500;
// ESP-IDF `MIPI_CSI_BRG_MEM_BASE`. The neighboring `0x5010_5000` aperture is
// the DSI bridge FIFO (`MIPI_DSI_BRG_MEM_BASE`), not CSI.
const CSI_BRG_MEM_BASE: u32 = 0x5010_4000;
const CSI_BRG_AFULL_THRESHOLD: u16 = 960;
const CSI_RAW8_DATA_TYPE: u8 = 0x2A;
// IDF CSI controller long-packet filter for YUV, RGB, and RAW capture.
const CSI_DATA_TYPE_MIN: u8 = 0x12;
const CSI_DATA_TYPE_MAX: u8 = 0x2F;

const DW_GDMA_TT_FC_P2M_SRC: u8 = 4;
const DW_GDMA_BURST_SIZE_512_ITEMS: u8 = 8;
const DW_GDMA_WIDTH_64_BITS: u8 = 3;
const DW_GDMA_AXI_BURST_LEN_16: u8 = 16;
const DW_GDMA_CHANNEL_PRIORITY: u8 = 1;
const DW_GDMA_OUTSTANDING_REQUESTS_5: u8 = 5;
const DW_GDMA_MAX_OUTSTANDING_REQUESTS: u8 = 16;
const DW_GDMA_HW_HANDSHAKE_PERIPH_CSI: u8 = 1;
// TRM v0.7 §7.5.7.2: after `CH_EN` is cleared, the channel stops reading the
// source only once the AXI reads it already issued there complete. The bridge
// may hold those until the next frame starts, so this wait covers more than
// one 30 fps frame period including vertical blanking.
const DMA_CHANNEL_DISABLE_TIMEOUT_MS: u64 = 50;
const DMA_CHANNEL_SUSPEND_TIMEOUT_MS: u64 = 5;
const DMA_CHANNEL_ABORT_TIMEOUT_MS: u64 = 5;
const CAPTURE_RUNTIME_SAMPLE_COUNT: usize = 4;
const CAPTURE_RUNTIME_SAMPLE_INTERVAL_MS: u64 = 25;
const CAPTURE_TRACE_CAPACITY: usize = 32;
const CAPTURE_TRACE_SHUTDOWN_ENTRIES: usize = 3;
const CAPTURE_TRACE_STALL_US: u64 = 2_000;

const DMA_STATUS_BLOCK_TRANSFER_DONE: u32 = 1 << 0;
const DMA_STATUS_DMA_TRANSFER_DONE: u32 = 1 << 1;
const DMA_STATUS_SRC_TRANSACTION_DONE: u32 = 1 << 3;
const DMA_STATUS_DST_TRANSACTION_DONE: u32 = 1 << 4;
const DMA_STATUS_CH_SRC_SUSPENDED: u32 = 1 << 28;
const DMA_STATUS_CH_SUSPENDED: u32 = 1 << 29;
const DMA_STATUS_CH_DISABLED: u32 = 1 << 30;
const DMA_STATUS_CH_ABORTED: u32 = 1 << 31;
const DMA_STATUS_ERROR_MASK: u32 = (1 << 5)
    | (1 << 6)
    | (1 << 7)
    | (1 << 8)
    | (1 << 9)
    | (1 << 10)
    | (1 << 11)
    | (1 << 12)
    | (1 << 13)
    | (1 << 14)
    | (1 << 16)
    | (1 << 17)
    | (1 << 18)
    | (1 << 19)
    | (1 << 20)
    | (1 << 21)
    | (1 << 25);
const DMA_STATUS_CHANNEL_STOPPED_MASK: u32 = DMA_STATUS_CH_DISABLED | DMA_STATUS_CH_ABORTED;
// The suspend/disable/abort bits only latch when enabled here. Shutdown needs
// them to report which stop step the channel completed.
const DMA_STATUS_ENABLE_MASK: u32 = DMA_STATUS_BLOCK_TRANSFER_DONE
    | DMA_STATUS_DMA_TRANSFER_DONE
    | DMA_STATUS_SRC_TRANSACTION_DONE
    | DMA_STATUS_DST_TRANSACTION_DONE
    | DMA_STATUS_ERROR_MASK
    | DMA_STATUS_CH_SRC_SUSPENDED
    | DMA_STATUS_CH_SUSPENDED
    | DMA_STATUS_CH_DISABLED
    | DMA_STATUS_CH_ABORTED;
// The statuses that end the polling wait. `DMA_STATUS_ENABLE_MASK` latches them all.
const DMA_COMPLETION_INTERRUPT_MASK: u32 =
    DMA_STATUS_BLOCK_TRANSFER_DONE | DMA_STATUS_DMA_TRANSFER_DONE | DMA_STATUS_ERROR_MASK;
const CSI_BRIDGE_INTERRUPT_MASK: u32 = 0x3F;
const DISCARD_DIAG_DAR_SAMPLES: usize = 8;

const fn csi_dma_channel_config(axi: DmaAxiConfig) -> ChannelConfig {
    ChannelConfig {
        src_multblk_type: 0,
        dst_multblk_type: 0,
        // P->M transfer with source (CSI peripheral) flow control.
        transfer_type_flow_control: DW_GDMA_TT_FC_P2M_SRC,
        src_handshake: HandshakeSelect::Hardware,
        dst_handshake: HandshakeSelect::Hardware,
        // DW-GDMA HW handshake peripheral selector: CSI=1 (DSI=0, CSI=1, ISP=2).
        src_peripheral: DW_GDMA_HW_HANDSHAKE_PERIPH_CSI,
        dst_peripheral: 0,
        // IDF configures the CSI bridge status aperture as source status-fetch
        // address for this source-controlled P2M transfer path.
        src_status_fetch_addr: CSI_BRG_MEM_BASE,
        dst_status_fetch_addr: 0,
        priority: DW_GDMA_CHANNEL_PRIORITY,
        // `CFG1.*_OSR_LMT` holds the request count minus one.
        src_outstanding_limit: axi.src_outstanding_requests - 1,
        dst_outstanding_limit: axi.dst_outstanding_requests - 1,
    }
}

/// AXI burst lengths and outstanding-request limits of the CSI DMA channel.
///
/// [`Self::ESP_IDF`] is used unless a bring-up diagnostic replaces it with
/// `CsiCapture::set_diagnostic_dma_axi_config`.
#[instability::unstable]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaAxiConfig {
    /// `CTL1.ARLEN` for reads from the CSI bridge FIFO, 1 to 16.
    ///
    /// The hardware reads a written 16 back as 15.
    pub src_burst_len: u8,
    /// `CTL1.AWLEN` for writes into the frame buffer, 1 to 16.
    ///
    /// The hardware reads a written 16 back as 15.
    pub dst_burst_len: u8,
    /// Outstanding read requests to the CSI bridge, 1 to 16.
    pub src_outstanding_requests: u8,
    /// Outstanding write requests to the frame buffer, 1 to 16.
    pub dst_outstanding_requests: u8,
}

impl DmaAxiConfig {
    /// Settings of the ESP-IDF v6.1 CSI driver (`esp_cam_ctlr_csi.c`).
    pub const ESP_IDF: Self = Self {
        src_burst_len: DW_GDMA_AXI_BURST_LEN_16,
        dst_burst_len: DW_GDMA_AXI_BURST_LEN_16,
        src_outstanding_requests: DW_GDMA_OUTSTANDING_REQUESTS_5,
        dst_outstanding_requests: DW_GDMA_OUTSTANDING_REQUESTS_5,
    };

    const fn is_valid(&self) -> bool {
        const fn in_range(value: u8, max: u8) -> bool {
            value >= 1 && value <= max
        }
        in_range(self.src_burst_len, DW_GDMA_AXI_BURST_LEN_16)
            && in_range(self.dst_burst_len, DW_GDMA_AXI_BURST_LEN_16)
            && in_range(
                self.src_outstanding_requests,
                DW_GDMA_MAX_OUTSTANDING_REQUESTS,
            )
            && in_range(
                self.dst_outstanding_requests,
                DW_GDMA_MAX_OUTSTANDING_REQUESTS,
            )
    }
}

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

/// Implemented by VDMA channel singletons (`VDMA_CH0`–`VDMA_CH3`) accepted by
/// [`MipiCsi::with_dma`].
pub trait CsiDmaChannel: Sealed + IntoOwnedCsiDmaChannel {
    /// Zero-based hardware channel index (0–3).
    fn channel_id(&self) -> u8;
}

/// Internal conversion to a type-erased owned VDMA channel token.
///
/// This trait is intentionally hidden and only public because
/// [`CsiDmaChannel`] is public.
#[doc(hidden)]
pub trait IntoOwnedCsiDmaChannel: Sealed {
    /// Converts the concrete channel singleton into a type-erased owned token.
    fn into_owned_csi_dma_channel<'d>(self) -> OwnedCsiDmaChannel<'d>
    where
        Self: Sized + 'd;
}

#[allow(non_camel_case_types)]
#[doc(hidden)]
#[allow(
    dead_code,
    reason = "Singleton token is intentionally held to preserve exclusive ownership"
)]
pub enum OwnedCsiDmaChannel<'d> {
    VDMA_CH0(crate::peripherals::VDMA_CH0<'d>),
    VDMA_CH1(crate::peripherals::VDMA_CH1<'d>),
    VDMA_CH2(crate::peripherals::VDMA_CH2<'d>),
    VDMA_CH3(crate::peripherals::VDMA_CH3<'d>),
}

impl OwnedCsiDmaChannel<'_> {
    #[inline]
    const fn channel_id(&self) -> u8 {
        match self {
            Self::VDMA_CH0(_) => 0,
            Self::VDMA_CH1(_) => 1,
            Self::VDMA_CH2(_) => 2,
            Self::VDMA_CH3(_) => 3,
        }
    }
}

for_each_dma_channel! {
    ("VDMA", $ch:ident, $num:literal, compatible = [$($compatible:ident),*]) => {
        impl IntoOwnedCsiDmaChannel for crate::peripherals::$ch<'_> {
            #[inline]
            fn into_owned_csi_dma_channel<'d>(self) -> OwnedCsiDmaChannel<'d>
            where
                Self: Sized + 'd,
            {
                OwnedCsiDmaChannel::$ch(self)
            }
        }

        impl CsiDmaChannel for crate::peripherals::$ch<'_> {
            #[inline]
            fn channel_id(&self) -> u8 {
                $num
            }
        }
    };
}

fn take_owned_csi_dma_channel<'d>(
    channel: impl CsiDmaChannel + 'd,
) -> (OwnedCsiDmaChannel<'d>, u8) {
    let owned: OwnedCsiDmaChannel<'d> = channel.into_owned_csi_dma_channel();
    let channel_id = owned.channel_id();
    (owned, channel_id)
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

/// Pixel format accepted by the CSI bridge capture path.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CapturePixelFormat {
    /// MIPI CSI-2 RAW8 (`data type = 0x2a`).
    Raw8,
}

impl CapturePixelFormat {
    #[inline]
    const fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Raw8 => 1,
        }
    }

    #[inline]
    const fn csi_data_type(self) -> u8 {
        match self {
            Self::Raw8 => CSI_RAW8_DATA_TYPE,
        }
    }
}

/// Single-frame CSI capture configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureConfig {
    /// Captured frame width in pixels.
    frame_width: u16,
    /// Captured frame height in lines.
    frame_height: u16,
    /// Pixel format expected from the sensor stream.
    pixel_format: CapturePixelFormat,
    /// Bounded timeout for blocking receive operations.
    timeout: Duration,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            frame_width: 640,
            frame_height: 480,
            pixel_format: CapturePixelFormat::Raw8,
            timeout: Duration::from_millis(100),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ValidatedCaptureConfig {
    frame_width: u16,
    frame_height: u16,
    frame_bytes: usize,
    transfer_size_units: u32,
    csi_data_type: u8,
    timeout: Duration,
}

impl CaptureConfig {
    fn validate(self) -> Result<ValidatedCaptureConfig, CaptureConfigError> {
        if self.frame_width == 0 || self.frame_height == 0 {
            return Err(CaptureConfigError::InvalidFrameDimensions);
        }

        // `frame_cfg` fields are 12-bit in the bridge register map.
        if self.frame_width > 0x0FFF || self.frame_height > 0x0FFF {
            return Err(CaptureConfigError::FrameDimensionsTooLarge);
        }

        let Some(frame_pixels) =
            (self.frame_width as usize).checked_mul(self.frame_height as usize)
        else {
            return Err(CaptureConfigError::FrameSizeOverflow);
        };
        let Some(frame_bytes) = frame_pixels.checked_mul(self.pixel_format.bytes_per_pixel())
        else {
            return Err(CaptureConfigError::FrameSizeOverflow);
        };

        if !frame_bytes.is_multiple_of(8) {
            return Err(CaptureConfigError::FrameSizeNotAlignedToTransferWidth);
        }

        let transfer_size_units = (frame_bytes / 8) as u32;
        if transfer_size_units == 0 || transfer_size_units > MAX_BLOCK_TRANSFER_SIZE_UNITS {
            return Err(CaptureConfigError::FrameTooLargeForSingleDmaBlock);
        }

        Ok(ValidatedCaptureConfig {
            frame_width: self.frame_width,
            frame_height: self.frame_height,
            frame_bytes,
            transfer_size_units,
            csi_data_type: self.pixel_format.csi_data_type(),
            timeout: self.timeout,
        })
    }
}

/// Capture configuration error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum CaptureConfigError {
    /// Frame width or height is zero.
    InvalidFrameDimensions,
    /// Frame geometry exceeds CSI bridge field limits.
    FrameDimensionsTooLarge,
    /// `width * height * bpp` overflowed.
    FrameSizeOverflow,
    /// The frame size is not divisible by 8 bytes while DMA is configured for 64-bit words.
    FrameSizeNotAlignedToTransferWidth,
    /// The frame transfer exceeds the DW-GDMA single-block limit.
    FrameTooLargeForSingleDmaBlock,
    /// A [`DmaAxiConfig`] field is outside its documented range.
    InvalidDmaAxiConfig,
}

impl fmt::Display for CaptureConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFrameDimensions => write!(f, "frame width and height must be non-zero"),
            Self::FrameDimensionsTooLarge => write!(
                f,
                "frame dimensions exceed CSI bridge limits (max 4095x4095)"
            ),
            Self::FrameSizeOverflow => write!(f, "frame size overflow"),
            Self::FrameSizeNotAlignedToTransferWidth => {
                write!(f, "frame size must be divisible by 8 bytes")
            }
            Self::FrameTooLargeForSingleDmaBlock => {
                write!(f, "frame exceeds DW-GDMA single-block transfer limit")
            }
            Self::InvalidDmaAxiConfig => {
                write!(
                    f,
                    "DMA AXI burst length or outstanding-request limit out of range"
                )
            }
        }
    }
}

impl core::error::Error for CaptureConfigError {}

/// DW-GDMA channel status snapshot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaStatus {
    /// Raw `INTSTATUS0` bits.
    pub raw_status: u32,
    /// Channel enable bit sampled from `CHEN0`.
    pub channel_enabled: bool,
    /// Completed transfer size from `STATUS0`, in transfer-width units.
    pub completed_transfer_units: u32,
    /// Remaining FIFO data from `STATUS1`, in transfer-width units.
    pub fifo_remaining_units: u16,
}

impl DmaStatus {
    /// Returns true if DMA transfer completion was reported.
    pub const fn transfer_done(&self) -> bool {
        (self.raw_status & (DMA_STATUS_BLOCK_TRANSFER_DONE | DMA_STATUS_DMA_TRANSFER_DONE)) != 0
    }

    /// Returns true if any DMA channel error bit is set.
    pub const fn any_error(&self) -> bool {
        (self.raw_status & DMA_STATUS_ERROR_MASK) != 0
    }

    /// Returns true if the channel reports disabled or aborted.
    pub const fn channel_stopped(&self) -> bool {
        (self.raw_status & DMA_STATUS_CHANNEL_STOPPED_MASK) != 0 || !self.channel_enabled
    }
}

/// Status snapshot captured during receive completion or failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureStatus {
    /// CSI host status bits.
    pub host_errors: HostErrorStatus,
    /// CSI bridge status bits.
    pub bridge_events: BridgeEventStatus,
    /// DMA channel status bits.
    pub dma_status: DmaStatus,
}

impl CaptureStatus {
    /// Returns true if CSI-side status indicates a receive failure.
    pub const fn csi_error(&self) -> bool {
        self.host_errors.any_error()
            || self.bridge_events.vadr_num_gt
            || self.bridge_events.vadr_num_lt
            || self.bridge_events.discard
            || self.bridge_events.csi_buf_overrun
            || self.bridge_events.csi_async_fifo_ovf
    }
}

/// Successful single-frame receive result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureFrame {
    /// Number of bytes the DMA wrote, measured from its destination address.
    ///
    /// The CSI bridge is the DMA flow controller and ends the block (TRM
    /// §7.5.5), so this differs from the configured frame size when the source
    /// frame does not match the configured geometry. `receive` only
    /// invalidates the cache for the configured frame size.
    pub received_bytes: usize,
}

/// Failure reason when CSI capture shutdown cannot be completed cleanly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ShutdownFailureReason {
    /// Timed out waiting for the DW-GDMA channel to complete an abort.
    AbortTimedOut,
}

/// Blocking receive error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ReceiveError {
    /// Buffer failed DMA alignment or memory-region validation.
    InvalidBuffer(DmaBufError),
    /// Buffer is smaller than the configured frame size.
    BufferTooSmall {
        /// Required frame size in bytes.
        required: usize,
        /// Provided buffer length in bytes.
        provided: usize,
    },
    /// No frame completed before timeout elapsed.
    Timeout(CaptureStatus),
    /// DMA channel reported an error.
    DmaError(CaptureStatus),
    /// CSI host/bridge reported a frame or protocol error.
    CsiError(CaptureStatus),
    /// Capture ended, but DMA channel shutdown did not complete cleanly.
    ShutdownFailed {
        /// Shutdown failure reason.
        reason: ShutdownFailureReason,
        /// Status sampled before shutdown was requested.
        status_before_shutdown: CaptureStatus,
        /// Status sampled after shutdown failure.
        status_after_shutdown: CaptureStatus,
    },
    /// [`CsiCapture::receive_with_interrupt`] did not arm, because another
    /// user of the shared DW-GDMA interrupt line may be active. Nothing was
    /// started and the buffer was not accessed.
    InterruptUnavailable {
        /// Other VDMA channels with `CH_EN` set, one bit per channel.
        enabled_channels: u32,
        /// Other sources driving the DW-GDMA interrupt line: bit `n` for
        /// channel `n`, bit 16 for the common register block.
        asserted_sources: u32,
    },
}

impl fmt::Display for ReceiveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidBuffer(err) => write!(f, "invalid DMA buffer: {err}"),
            Self::BufferTooSmall { required, provided } => write!(
                f,
                "buffer too small for frame (required {required} bytes, provided {provided} bytes)"
            ),
            Self::Timeout(_) => write!(f, "timed out waiting for CSI frame"),
            Self::DmaError(_) => write!(f, "DMA channel reported an error during capture"),
            Self::CsiError(_) => write!(f, "CSI host/bridge reported an error during capture"),
            Self::ShutdownFailed { reason, .. } => {
                write!(
                    f,
                    "failed to stop CSI DMA channel safely after capture ({reason:?})"
                )
            }
            Self::InterruptUnavailable {
                enabled_channels,
                asserted_sources,
            } => write!(
                f,
                "DW-GDMA interrupt line in use (enabled channels 0x{enabled_channels:x}, asserted sources 0x{asserted_sources:x})"
            ),
        }
    }
}

impl core::error::Error for ReceiveError {}

/// D-PHY configuration clock source.
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
    /// Raw `int_raw` value for debugging.
    pub raw_status: u32,
}

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

/// Result of the temporary `DISCARD` diagnostic.
///
/// `dma_completed` means the programmed DW-GDMA block finished. That is not the
/// same as a frame-aligned image: a startup `CSI_BRIG_DISCARD` means the block
/// began on a partial frame, and this diagnostic does not re-arm the destination
/// at the next frame boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiscardContinueDiagnostic {
    /// The diagnostic was explicitly enabled for this capture.
    pub enabled: bool,
    /// 1 if `CSI_BRIG_DISCARD` was raised during the capture.
    ///
    /// The bridge interrupt bits are not cleared while a transfer runs, so a
    /// second discard in the same capture cannot be told apart from the first.
    pub discard_count: u32,
    /// A non-discard CSI/DMA failure ended the wait.
    pub fatal_error_seen: bool,
    /// The programmed DMA block reported completion.
    pub dma_completed: bool,
    /// DMA completed with no discard and no fatal bridge/host error.
    ///
    /// This stays false when any discard was counted. The bytes written after a
    /// discarded fragment are not treated as one valid frame.
    pub valid_complete_frame: bool,
    /// `DAR` immediately after the DMA channel was armed.
    pub dar_start: u32,
    /// `DAR` at the end of the capture wait, before shutdown.
    pub dar_end: u32,
    /// Destination-address samples taken at start, during the wait, on discard, and at the end.
    pub dar_samples: [Option<u32>; DISCARD_DIAG_DAR_SAMPLES],
    /// `STATUS0` completed units at the end of the wait.
    pub completed_transfer_units: u32,
    /// Programmed block size in 64-bit units.
    pub expected_transfer_units: u32,
}

#[derive(Clone, Copy, Debug)]
struct DiscardDiag {
    armed: bool,
    discard_latched: bool,
    discard_count: u32,
    fatal_error_seen: bool,
    dma_completed: bool,
    dar_start: u32,
    dar_end: u32,
    dar_samples: [Option<u32>; DISCARD_DIAG_DAR_SAMPLES],
    completed_units: u32,
}

impl DiscardDiag {
    const fn idle() -> Self {
        Self {
            armed: false,
            discard_latched: false,
            discard_count: 0,
            fatal_error_seen: false,
            dma_completed: false,
            dar_start: 0,
            dar_end: 0,
            dar_samples: [None; DISCARD_DIAG_DAR_SAMPLES],
            completed_units: 0,
        }
    }

    fn reset_for_capture(&mut self, dar_start: u32) {
        let armed = self.armed;
        *self = Self::idle();
        self.armed = armed;
        self.dar_start = dar_start;
        self.dar_end = dar_start;
        if armed {
            self.push_dar(dar_start);
        }
    }

    fn push_dar(&mut self, dar: u32) {
        if self
            .dar_samples
            .iter()
            .flatten()
            .next_back()
            .is_some_and(|previous| *previous == dar)
        {
            return;
        }
        if let Some(slot) = self.dar_samples.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(dar);
        }
    }
}

/// Runtime hardware status that is not just static configuration readback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HardwareStatus {
    /// CSI host error summary.
    pub host_errors: HostErrorStatus,
    /// CSI bridge event summary.
    pub bridge_events: BridgeEventStatus,
}

/// Full initialization diagnostics snapshot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InitDiagnostics {
    /// Configuration register readback.
    pub config: ConfigReadback,
    /// Runtime status/event readback.
    pub status: HardwareStatus,
}

/// Low-level CSI/DMA snapshot for hardware bring-up diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureDebugSnapshot {
    /// Active VDMA channel id.
    pub dma_channel_id: u8,
    /// `MIPI_CSI_HOST.phy_stopstate`
    pub host_phy_stopstate_raw: u32,
    /// `MIPI_CSI_HOST.phy_rx`
    pub host_phy_rx_raw: u32,
    /// `MIPI_CSI_HOST.int_st_main`
    pub host_int_st_main_raw: u32,
    /// `MIPI_CSI_BRIDGE.csi_en`
    pub bridge_csi_en_raw: u32,
    /// `MIPI_CSI_BRIDGE.frame_cfg`
    pub bridge_frame_cfg_raw: u32,
    /// `MIPI_CSI_BRIDGE.data_type_cfg`
    pub bridge_data_type_cfg_raw: u32,
    /// `MIPI_CSI_BRIDGE.host_cm_ctrl`
    pub bridge_host_cm_ctrl_raw: u32,
    /// `MIPI_CSI_BRIDGE.buf_flow_ctl`
    pub bridge_buf_flow_ctl_raw: u32,
    /// `MIPI_CSI_BRIDGE.dma_req_cfg`
    pub bridge_dma_req_cfg_raw: u32,
    /// `MIPI_CSI_BRIDGE.dmablk_size`
    pub bridge_dmablk_size_raw: u32,
    /// `MIPI_CSI_BRIDGE.int_raw`
    pub bridge_int_raw_raw: u32,
    /// `MIPI_CSI_BRIDGE.int_st`
    pub bridge_int_st_raw: u32,
    /// `MIPI_CSI_BRIDGE.int_ena`
    pub bridge_int_ena_raw: u32,
    /// `VDMA.CHEN0`
    pub dma_chen0_raw: u32,
    /// `VDMA.CHEN1`
    pub dma_chen1_raw: u32,
    /// `VDMA.CFG0`
    pub dma_controller_cfg0_raw: u32,
    /// `VDMA.RESET0`
    pub dma_controller_reset0_raw: u32,
    /// `VDMA.LOWPOWER_CFG0`
    pub dma_controller_lowpower_cfg0_raw: u32,
    /// `VDMA.LOWPOWER_CFG1`
    pub dma_controller_lowpower_cfg1_raw: u32,
    /// `VDMA.CHx_CFG0`
    pub dma_cfg0_raw: u32,
    /// `VDMA.CHx_CFG1`
    pub dma_cfg1_raw: u32,
    /// `VDMA.CHx_CTL0`
    pub dma_ctl0_raw: u32,
    /// `VDMA.CHx_CTL1`
    pub dma_ctl1_raw: u32,
    /// `VDMA.CHx_SAR0`
    pub dma_sar0_raw: u32,
    /// `VDMA.CHx_DAR0`
    pub dma_dar0_raw: u32,
    /// `VDMA.CHx_BLOCK_TS0`
    pub dma_block_ts0_raw: u32,
    /// `VDMA.CHx_SSTATAR0`
    pub dma_sstatar0_raw: u32,
    /// `VDMA.CHx_DSTATAR0`
    pub dma_dstatar0_raw: u32,
    /// `VDMA.CHx_SSTAT0`
    pub dma_sstat0_raw: u32,
    /// `VDMA.CHx_DSTAT0`
    pub dma_dstat0_raw: u32,
    /// `VDMA.CHx_STATUS0`
    pub dma_status0_raw: u32,
    /// `VDMA.CHx_STATUS1`
    pub dma_status1_raw: u32,
    /// `VDMA.CHx_INTSTATUS_ENABLE0`
    pub dma_intstatus_enable0_raw: u32,
    /// `VDMA.CHx_INTSTATUS0`
    pub dma_intstatus0_raw: u32,
}

/// Aggregated D-PHY runtime activity sampled during `receive`.
///
/// Sampling is performed inside the capture polling loop. `*_seen` flags are
/// only updated for samples where both CSI bridge and DMA channel are enabled,
/// so they reflect activity during an armed capture window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CapturePhyActivitySummary {
    /// Total number of loop samples.
    pub sample_count: u32,
    /// Samples where `csi_brig_en` and DMA channel-enable were both set.
    pub active_sample_count: u32,
    /// Samples where bridge-enable bit was set.
    pub bridge_enabled_sample_count: u32,
    /// Samples where DMA channel-enable bit was set.
    pub dma_enabled_sample_count: u32,
    /// Bitwise OR accumulation of `phy_stopstate`.
    pub phy_stopstate_or_raw: u32,
    /// Bitwise AND accumulation of `phy_stopstate`.
    pub phy_stopstate_and_raw: u32,
    /// Bitwise OR accumulation of `phy_rx`.
    pub phy_rx_or_raw: u32,
    /// Bitwise AND accumulation of `phy_rx`.
    pub phy_rx_and_raw: u32,
    /// Bridge FIFO depth was non-zero at least once during active capture.
    pub bridge_buf_depth_nonzero_seen: bool,
    /// Maximum observed bridge FIFO depth during active capture.
    pub bridge_buf_depth_max: u16,
    /// `phy_rx.rxclkactivehs` was observed set during active capture.
    pub clkactivehs_seen: bool,
    /// Data lane0 left stop-state during active capture.
    pub data_lane0_left_stopstate_seen: bool,
    /// Data lane1 left stop-state during active capture.
    pub data_lane1_left_stopstate_seen: bool,
}

impl CapturePhyActivitySummary {
    const fn empty() -> Self {
        Self {
            sample_count: 0,
            active_sample_count: 0,
            bridge_enabled_sample_count: 0,
            dma_enabled_sample_count: 0,
            phy_stopstate_or_raw: 0,
            phy_stopstate_and_raw: 0,
            phy_rx_or_raw: 0,
            phy_rx_and_raw: 0,
            bridge_buf_depth_nonzero_seen: false,
            bridge_buf_depth_max: 0,
            clkactivehs_seen: false,
            data_lane0_left_stopstate_seen: false,
            data_lane1_left_stopstate_seen: false,
        }
    }

    #[expect(clippy::too_many_arguments)]
    fn record_sample(
        &mut self,
        phy_stopstate_raw: u32,
        phy_rx_raw: u32,
        bridge_enabled: bool,
        dma_enabled: bool,
        bridge_buf_depth: u16,
        clkactivehs: bool,
        data_lane0_stopstate: bool,
        data_lane1_stopstate: bool,
    ) {
        if self.sample_count == 0 {
            self.phy_stopstate_or_raw = phy_stopstate_raw;
            self.phy_stopstate_and_raw = phy_stopstate_raw;
            self.phy_rx_or_raw = phy_rx_raw;
            self.phy_rx_and_raw = phy_rx_raw;
        } else {
            self.phy_stopstate_or_raw |= phy_stopstate_raw;
            self.phy_stopstate_and_raw &= phy_stopstate_raw;
            self.phy_rx_or_raw |= phy_rx_raw;
            self.phy_rx_and_raw &= phy_rx_raw;
        }

        self.sample_count = self.sample_count.saturating_add(1);
        if bridge_enabled {
            self.bridge_enabled_sample_count = self.bridge_enabled_sample_count.saturating_add(1);
        }
        if dma_enabled {
            self.dma_enabled_sample_count = self.dma_enabled_sample_count.saturating_add(1);
        }

        if bridge_enabled && dma_enabled {
            self.active_sample_count = self.active_sample_count.saturating_add(1);
            self.bridge_buf_depth_nonzero_seen |= bridge_buf_depth != 0;
            self.bridge_buf_depth_max = self.bridge_buf_depth_max.max(bridge_buf_depth);
            self.clkactivehs_seen |= clkactivehs;
            self.data_lane0_left_stopstate_seen |= !data_lane0_stopstate;
            self.data_lane1_left_stopstate_seen |= !data_lane1_stopstate;
        }
    }
}

/// Reason a `CaptureTraceEntry` was recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CaptureTraceEvent {
    /// The DMA channel and the CSI bridge were enabled.
    Armed,
    /// The `start_source` callback of [`CsiCapture::receive_with`] returned.
    SourceStarted,
    /// The DMA destination address moved for the first time.
    FirstData,
    /// New bits were set in the bridge `int_raw` register.
    BridgeEvent,
    /// New bits were set in the channel `INTSTATUS0` register.
    DmaEvent,
    /// The destination address stopped moving for at least 2 ms.
    Stalled,
    /// The destination address moved again after a stall.
    Resumed,
    /// The wait for the frame ended, before shutdown.
    WaitEnded,
    /// `CH_EN` read back clear after the channel was disabled.
    ChannelDisabled,
    /// The channel only stopped after suspend and abort.
    ChannelAborted,
    /// The channel could not be shown to be stopped.
    ShutdownFailed,
    /// The CSI bridge was disabled after the channel stop attempt.
    BridgeDisabled,
}

/// One entry of the capture timeline recorded by `receive`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureTraceEntry {
    /// Why this entry was recorded.
    pub event: CaptureTraceEvent,
    /// Microseconds since the channel was armed.
    pub elapsed_us: u32,
    /// `VDMA.CHx_DAR0`
    pub dar: u32,
    /// `MIPI_CSI_BRIDGE.int_raw`
    pub bridge_int_raw: u32,
    /// `MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_depth`, in 64-bit words.
    pub bridge_fifo_depth: u16,
    /// `VDMA.CHx_INTSTATUS0`
    pub dma_intstatus: u32,
    /// `CH_EN` of the capture channel.
    pub dma_enabled: bool,
    /// `CH_ABORT` of the capture channel is still set.
    pub dma_abort_pending: bool,
    /// `MIPI_CSI_HOST.phy_stopstate`
    pub phy_stopstate: u32,
}

impl CaptureTraceEntry {
    const EMPTY: Self = Self {
        event: CaptureTraceEvent::Armed,
        elapsed_us: 0,
        dar: 0,
        bridge_int_raw: 0,
        bridge_fifo_depth: 0,
        dma_intstatus: 0,
        dma_enabled: false,
        dma_abort_pending: false,
        phy_stopstate: 0,
    };
}

/// Bounded timeline of the previous `receive`.
///
/// Entries are only recorded when something changes, so a stalled transfer
/// and a completed one both fit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CaptureTrace {
    entries: [CaptureTraceEntry; CAPTURE_TRACE_CAPACITY],
    len: usize,
    /// Entries that did not fit.
    pub dropped: u32,
    /// Destination address programmed for the capture.
    pub dar_start: u32,
    /// Number of iterations of the wait loop.
    pub polls: u32,
}

impl CaptureTrace {
    const fn empty() -> Self {
        Self {
            entries: [CaptureTraceEntry::EMPTY; CAPTURE_TRACE_CAPACITY],
            len: 0,
            dropped: 0,
            dar_start: 0,
            polls: 0,
        }
    }

    /// Recorded entries, oldest first.
    pub fn entries(&self) -> &[CaptureTraceEntry] {
        &self.entries[..self.len]
    }

    fn push(&mut self, entry: CaptureTraceEntry) {
        // Events recorded while waiting leave room for the ones recorded after
        // the wait, which describe the shutdown.
        let limit = match entry.event {
            CaptureTraceEvent::WaitEnded
            | CaptureTraceEvent::ChannelDisabled
            | CaptureTraceEvent::ChannelAborted
            | CaptureTraceEvent::ShutdownFailed
            | CaptureTraceEvent::BridgeDisabled => CAPTURE_TRACE_CAPACITY,
            _ => CAPTURE_TRACE_CAPACITY - CAPTURE_TRACE_SHUTDOWN_ENTRIES,
        };
        if self.len < limit {
            self.entries[self.len] = entry;
            self.len += 1;
        } else {
            self.dropped = self.dropped.saturating_add(1);
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct CaptureTraceRecorder {
    armed_at: Instant,
    last_dar: u32,
    last_dar_change: Instant,
    first_data_seen: bool,
    stalled: bool,
    last_bridge_int_raw: u32,
    last_dma_intstatus: u32,
}

impl CaptureTraceRecorder {
    const fn idle() -> Self {
        Self {
            armed_at: Instant::EPOCH,
            last_dar: 0,
            last_dar_change: Instant::EPOCH,
            first_data_seen: false,
            stalled: false,
            last_bridge_int_raw: 0,
            last_dma_intstatus: 0,
        }
    }
}

/// What the completion interrupt of the last
/// [`CsiCapture::receive_with_interrupt`] observed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CompletionInterruptReport {
    /// The handler ran before the wait ended.
    pub fired: bool,
    /// The capture channel raised it. `false` with `fired` means another
    /// source drove the shared line, and the wait fell back to polling.
    pub own_source: bool,
    /// Channel `INTSTATUS0` read by the handler.
    pub channel_status: u32,
    /// Microseconds from arming the interrupt, just before the channel is
    /// enabled, to the handler.
    pub latency_from_arm_us: Option<u32>,
}

/// MIPI CSI controller handle.
///
/// Owns the `MIPI_CSI` virtual peripheral singleton and keeps HOST/BRIDGE
/// clocks enabled for as long as this value lives.
pub struct MipiCsi<'d> {
    _csi: MIPI_CSI<'d>,
    _guard: GenericPeripheralGuard<{ Peripheral::MipiCsi as u8 }>,
}

/// Blocking single-frame CSI capture handle.
///
/// This owns one dedicated VDMA channel and keeps the channel configured for
/// CSI peripheral-to-memory transfers.
pub struct CsiCapture<'d> {
    csi: MipiCsi<'d>,
    config: ValidatedCaptureConfig,
    dma_channel: DwGdmaChannel,
    dma_axi: DmaAxiConfig,
    _vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }>,
    _owned_dma_channel: OwnedCsiDmaChannel<'d>,
    runtime_samples: [Option<CaptureStatus>; CAPTURE_RUNTIME_SAMPLE_COUNT],
    runtime_phy_activity: CapturePhyActivitySummary,
    trace: CaptureTrace,
    trace_recorder: CaptureTraceRecorder,
    /// Temporary bring-up switch. Default `armed = false` keeps `DISCARD` fatal.
    discard_diag: DiscardDiag,
    completion_interrupt: CompletionInterruptReport,
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
        // Bridge host clocks must be running before the D-PHY test-interface
        // write. IDF claims the bridge, and enables `csi_enableclk`, before
        // `mipi_csi_hal_init`.
        this.prepare_bridge();
        this.init_host(config);

        Ok(this)
    }

    /// Attaches one VDMA channel and prepares a blocking single-frame capture
    /// path.
    pub fn with_dma(
        self,
        channel: impl CsiDmaChannel + 'd,
        config: CaptureConfig,
    ) -> Result<CsiCapture<'d>, CaptureConfigError> {
        let config = config.validate()?;
        let (owned_dma_channel, channel_id) = take_owned_csi_dma_channel(channel);

        let vdma_guard = GenericPeripheralGuard::new();
        enable_controller();

        let dma_channel = DwGdmaChannel::new(channel_id);
        dma_channel.set_enabled(false);
        dma_channel.configure(csi_dma_channel_config(DmaAxiConfig::ESP_IDF));
        dma_channel.set_interrupt_status_enable(DMA_STATUS_ENABLE_MASK);
        dma_channel.set_interrupt_signal_enable(0);
        dma_channel.clear_all_interrupts();

        self.configure_bridge_capture(config);
        self.clear_bridge_interrupts();

        Ok(CsiCapture {
            csi: self,
            config,
            dma_channel,
            dma_axi: DmaAxiConfig::ESP_IDF,
            _vdma_guard: vdma_guard,
            _owned_dma_channel: owned_dma_channel,
            runtime_samples: [None; CAPTURE_RUNTIME_SAMPLE_COUNT],
            runtime_phy_activity: CapturePhyActivitySummary::empty(),
            trace: CaptureTrace::empty(),
            trace_recorder: CaptureTraceRecorder::idle(),
            discard_diag: DiscardDiag::idle(),
            completion_interrupt: CompletionInterruptReport::default(),
        })
    }

    /// Returns a readback snapshot of initialization-related CSI state.
    ///
    /// Register readback is useful to verify software configuration, but it
    /// does not prove a working MIPI CSI physical link.
    pub fn init_diagnostics(&self) -> InitDiagnostics {
        let host = MIPI_CSI_HOST::regs();
        let bridge = MIPI_CSI_BRIDGE::regs();
        let clkrst = HP_SYS_CLKRST::regs();

        let host_clk = clkrst.soc_clk_ctrl1().read();
        let rst = clkrst.hp_rst_en0().read();
        let phy_cfg_clk = clkrst.peri_clk_ctrl03().read();
        let bridge_csi_en = bridge.csi_en().read();

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
                host_errors: self.read_host_error_status(),
                bridge_events: self.read_bridge_event_status(),
            },
        }
    }

    fn init_phy_cfg_clock(&self) {
        // IDF `esp_clk_tree_enable_src(MIPI_CSI_PHY_CLK_SRC_PLL_F20M)` opens
        // this gate before any D-PHY test-interface access. The P4 clock
        // driver leaves `enable_pll_f20m` as a stub, so CSI has to do it.
        HP_SYS_CLKRST::regs()
            .ref_clk_ctrl2()
            .modify(|_, w| w.ref_20m_clk_en().set_bit());

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

    fn prepare_bridge(&self) {
        let bridge = MIPI_CSI_BRIDGE::regs();

        // Pulse bridge reset before the D-PHY test-interface write. Doing it
        // afterwards can clear `csi_enableclk`, which gates the PHY clock-lane
        // module. `csi_brig_en` stays clear until capture starts.
        bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_rst().set_bit().csi_brig_en().clear_bit());
        bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_rst().clear_bit().csi_brig_en().clear_bit());
        bridge.host_ctrl().modify(|_, w| {
            w.csi_enableclk().set_bit();
            w.csi_cfg_clk_en().set_bit()
        });
        bridge.int_ena().reset();
    }

    fn configure_bridge_capture(&self, config: ValidatedCaptureConfig) {
        let bridge = MIPI_CSI_BRIDGE::regs();

        bridge
            .buf_flow_ctl()
            .modify(|_, w| unsafe { w.csi_buf_afull_thrd().bits(CSI_BRG_AFULL_THRESHOLD) });
        debug_assert!(
            (CSI_DATA_TYPE_MIN..=CSI_DATA_TYPE_MAX).contains(&config.csi_data_type),
            "capture data type is outside the CSI bridge packet filter"
        );
        bridge.data_type_cfg().write(|w| unsafe {
            w.data_type_min().bits(CSI_DATA_TYPE_MIN);
            w.data_type_max().bits(CSI_DATA_TYPE_MAX)
        });
        // Rev3 RAW8 path: enable the bridge color-mode block and bypass it
        // when input and output formats match. IDF does this in
        // `s_csi_ctlr_format_conversion` for every equal-format capture.
        bridge.host_cm_ctrl().modify(|_, w| {
            w.csi_host_cm_en().set_bit();
            w.csi_host_cm_bypass().set_bit()
        });
        bridge.frame_cfg().modify(|_, w| unsafe {
            w.vadr_num().bits(config.frame_height);
            w.hadr_num().bits(config.frame_width)
        });
        bridge.dma_req_cfg().modify(|_, w| unsafe {
            // Keep bridge-side flow control (default=1) and use 512x64-bit
            // request bursts to match Espressif's CSI reference driver setup.
            w.dma_burst_len().bits(512);
            w.dma_cfg_upd_by_blk().clear_bit();
            w.dma_force_rd_status().clear_bit();
            w.csi_dma_flow_controller().set_bit()
        });
        // `int_ena` stays at its reset value of 0, as in IDF, which never
        // writes the bridge interrupt registers. Events are read from `int_raw`.
    }

    /// Clears the bridge event bits. Only called while the bridge is disabled.
    fn clear_bridge_interrupts(&self) {
        let bridge = MIPI_CSI_BRIDGE::regs();
        unsafe {
            bridge
                .int_clr()
                .write(|w| w.bits(CSI_BRIDGE_INTERRUPT_MASK))
        };
    }

    fn read_host_error_status(&self) -> HostErrorStatus {
        let host_int_main = MIPI_CSI_HOST::regs().int_st_main().read();
        HostErrorStatus {
            phy_fatal: host_int_main.st_status_int_phy_fatal().bit_is_set(),
            packet_fatal: host_int_main.st_status_int_pkt_fatal().bit_is_set(),
            boundary_frame_fatal: host_int_main.st_status_int_bndry_frame_fatal().bit_is_set(),
            sequence_frame_fatal: host_int_main.st_status_int_seq_frame_fatal().bit_is_set(),
            crc_frame_fatal: host_int_main.st_status_int_crc_frame_fatal().bit_is_set(),
            payload_crc_fatal: host_int_main.st_status_int_pld_crc_fatal().bit_is_set(),
            data_id_error: host_int_main.st_status_int_data_id().bit_is_set(),
            ecc_corrected: host_int_main.st_status_int_ecc_corrected().bit_is_set(),
            phy_error: host_int_main.st_status_int_phy().bit_is_set(),
            raw_main_status: host_int_main.bits(),
        }
    }

    fn read_bridge_event_status(&self) -> BridgeEventStatus {
        let bridge_int = MIPI_CSI_BRIDGE::regs().int_raw().read();
        BridgeEventStatus {
            vadr_num_gt: bridge_int.vadr_num_gt().bit_is_set(),
            vadr_num_lt: bridge_int.vadr_num_lt().bit_is_set(),
            discard: bridge_int.discard().bit_is_set(),
            csi_buf_overrun: bridge_int.csi_buf_overrun().bit_is_set(),
            csi_async_fifo_ovf: bridge_int.csi_async_fifo_ovf().bit_is_set(),
            dma_cfg_has_updated: bridge_int.dma_cfg_has_updated().bit_is_set(),
            raw_status: bridge_int.bits(),
        }
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

impl CsiCapture<'_> {
    /// Returns a low-level CSI/DMA register snapshot for bring-up diagnostics.
    pub fn debug_snapshot(&self) -> CaptureDebugSnapshot {
        let host = MIPI_CSI_HOST::regs();
        let bridge = MIPI_CSI_BRIDGE::regs();
        let dma = self.dma_channel.debug_registers();

        CaptureDebugSnapshot {
            dma_channel_id: self._owned_dma_channel.channel_id(),
            host_phy_stopstate_raw: host.phy_stopstate().read().bits(),
            host_phy_rx_raw: host.phy_rx().read().bits(),
            host_int_st_main_raw: host.int_st_main().read().bits(),
            bridge_csi_en_raw: bridge.csi_en().read().bits(),
            bridge_frame_cfg_raw: bridge.frame_cfg().read().bits(),
            bridge_data_type_cfg_raw: bridge.data_type_cfg().read().bits(),
            bridge_host_cm_ctrl_raw: bridge.host_cm_ctrl().read().bits(),
            bridge_buf_flow_ctl_raw: bridge.buf_flow_ctl().read().bits(),
            bridge_dma_req_cfg_raw: bridge.dma_req_cfg().read().bits(),
            bridge_dmablk_size_raw: bridge.dmablk_size().read().bits(),
            bridge_int_raw_raw: bridge.int_raw().read().bits(),
            bridge_int_st_raw: bridge.int_st().read().bits(),
            bridge_int_ena_raw: bridge.int_ena().read().bits(),
            dma_chen0_raw: dma.chen0,
            dma_chen1_raw: dma.chen1,
            dma_controller_cfg0_raw: dma.controller_cfg0,
            dma_controller_reset0_raw: dma.controller_reset0,
            dma_controller_lowpower_cfg0_raw: dma.controller_lowpower_cfg0,
            dma_controller_lowpower_cfg1_raw: dma.controller_lowpower_cfg1,
            dma_cfg0_raw: dma.cfg0,
            dma_cfg1_raw: dma.cfg1,
            dma_ctl0_raw: dma.ctl0,
            dma_ctl1_raw: dma.ctl1,
            dma_sar0_raw: dma.sar0,
            dma_dar0_raw: dma.dar0,
            dma_block_ts0_raw: dma.block_ts0,
            dma_sstatar0_raw: dma.sstatar0,
            dma_dstatar0_raw: dma.dstatar0,
            dma_sstat0_raw: dma.sstat0,
            dma_dstat0_raw: dma.dstat0,
            dma_status0_raw: dma.status0,
            dma_status1_raw: dma.status1,
            dma_intstatus_enable0_raw: dma.intstatus_enable0,
            dma_intstatus0_raw: dma.intstatus0,
        }
    }

    /// Returns periodic status samples collected during the previous `receive` call.
    pub fn runtime_status_samples(&self) -> &[Option<CaptureStatus>] {
        &self.runtime_samples
    }

    /// Returns aggregated D-PHY runtime activity from the previous `receive`.
    pub fn runtime_phy_activity_summary(&self) -> CapturePhyActivitySummary {
        self.runtime_phy_activity
    }

    /// Returns the timeline recorded during the previous `receive`.
    pub fn capture_trace(&self) -> &CaptureTrace {
        &self.trace
    }

    /// Overrides only the DMA block length used by the next `receive`.
    ///
    /// The CSI bridge pixel geometry programmed by `with_dma` is left unchanged.
    /// `receive` refuses a buffer shorter than `bytes`, and the DW-GDMA block
    /// size is this length, so the destination is not expected to advance past it.
    pub fn set_diagnostic_dma_transfer_bytes(
        &mut self,
        bytes: usize,
    ) -> Result<(), CaptureConfigError> {
        if bytes == 0 || !bytes.is_multiple_of(8) {
            return Err(CaptureConfigError::FrameSizeNotAlignedToTransferWidth);
        }
        let units = (bytes / 8) as u32;
        if units > MAX_BLOCK_TRANSFER_SIZE_UNITS {
            return Err(CaptureConfigError::FrameTooLargeForSingleDmaBlock);
        }
        self.config.frame_bytes = bytes;
        self.config.transfer_size_units = units;
        Ok(())
    }

    /// Replaces the [`DmaAxiConfig::ESP_IDF`] AXI settings used by the next
    /// `receive`.
    ///
    /// Flow control, handshake, addressing, transfer width and burst items stay
    /// as ESP-IDF programs them.
    pub fn set_diagnostic_dma_axi_config(
        &mut self,
        config: DmaAxiConfig,
    ) -> Result<(), CaptureConfigError> {
        if !config.is_valid() {
            return Err(CaptureConfigError::InvalidDmaAxiConfig);
        }
        // `CFG1` may only be written while the channel is disabled. `receive`
        // either leaves the channel stopped or panics.
        debug_assert!(!self.dma_channel.is_enabled());
        self.dma_axi = config;
        self.dma_channel.configure(csi_dma_channel_config(config));
        Ok(())
    }

    /// Returns the AXI settings the next `receive` programs.
    pub fn dma_axi_config(&self) -> DmaAxiConfig {
        self.dma_axi
    }

    /// Enables the temporary diagnostic that continues after `CSI_BRIG_DISCARD`.
    ///
    /// Default capture behavior is unchanged until this is called. Overrun,
    /// asynchronous-FIFO overflow, and vertical-count errors still stop the wait.
    /// This does not resynchronize DMA to the next frame boundary.
    pub fn enable_discard_continue_diagnostic(&mut self) {
        self.discard_diag.armed = true;
    }

    /// Returns the last discard-continue diagnostic result.
    pub fn discard_continue_diagnostic(&self) -> DiscardContinueDiagnostic {
        DiscardContinueDiagnostic {
            enabled: self.discard_diag.armed,
            discard_count: self.discard_diag.discard_count,
            fatal_error_seen: self.discard_diag.fatal_error_seen,
            dma_completed: self.discard_diag.dma_completed,
            valid_complete_frame: self.discard_diag.armed
                && self.discard_diag.dma_completed
                && self.discard_diag.discard_count == 0
                && !self.discard_diag.fatal_error_seen,
            dar_start: self.discard_diag.dar_start,
            dar_end: self.discard_diag.dar_end,
            dar_samples: self.discard_diag.dar_samples,
            completed_transfer_units: self.discard_diag.completed_units,
            expected_transfer_units: self.config.transfer_size_units,
        }
    }

    /// Receives one CSI frame into `buffer` and blocks until completion or timeout.
    ///
    /// `buffer` must be DMA-safe (alignment + memory region) and at least as
    /// large as the configured frame size.
    ///
    /// # Panics
    ///
    /// Panics if DW-GDMA shutdown cannot prove the channel is quiesced after a
    /// timeout/error path. This prevents returning buffer ownership to safe
    /// Rust while DMA may still write into it. Also panics if the DMA
    /// destination address ended up past the end of `buffer`.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<CaptureFrame, ReceiveError> {
        self.receive_with(buffer, || {})
    }

    /// Arms the DMA channel and the CSI bridge, calls `start_source`, then
    /// blocks until one frame completes or the timeout expires.
    ///
    /// Espressif's esp-video framework enables the sensor stream only after the
    /// CSI controller, the ISP and the DMA are running, so the bridge starts on
    /// a frame boundary. Enable the sensor stream in `start_source` to follow
    /// the same order. The timeout starts when `start_source` returns.
    ///
    /// # Panics
    ///
    /// Same as [`Self::receive`].
    pub fn receive_with(
        &mut self,
        buffer: &mut [u8],
        start_source: impl FnOnce(),
    ) -> Result<CaptureFrame, ReceiveError> {
        self.receive_impl(buffer, start_source, CompletionWait::Polling)
    }

    /// Same as [`Self::receive_with`], but waits for the DW-GDMA completion
    /// interrupt instead of polling the hardware during the frame.
    ///
    /// The interrupt only ends the wait. The capture is then classified from
    /// the same status registers, and the channel is stopped and the buffer
    /// handed back exactly as in [`Self::receive_with`]: a completion
    /// interrupt is not taken as proof that the channel stopped writing.
    ///
    /// All VDMA channels share [`Interrupt::DMA`](crate::peripherals::Interrupt::DMA). Its handler
    /// hands each channel only its own interrupt sources, and this method only masks the
    /// sources of the capture channel. It still returns
    /// [`ReceiveError::InterruptUnavailable`] without starting anything if
    /// another VDMA channel is enabled or already drives the line.
    ///
    /// # Panics
    ///
    /// Same as [`Self::receive`].
    pub fn receive_with_interrupt(
        &mut self,
        buffer: &mut [u8],
        start_source: impl FnOnce(),
    ) -> Result<CaptureFrame, ReceiveError> {
        self.receive_impl(buffer, start_source, CompletionWait::Interrupt)
    }

    /// Returns what the completion interrupt observed during the last
    /// [`Self::receive_with_interrupt`].
    pub fn completion_interrupt_report(&self) -> CompletionInterruptReport {
        self.completion_interrupt
    }

    fn receive_impl(
        &mut self,
        buffer: &mut [u8],
        start_source: impl FnOnce(),
        wait: CompletionWait,
    ) -> Result<CaptureFrame, ReceiveError> {
        if buffer.len() < self.config.frame_bytes {
            return Err(ReceiveError::BufferTooSmall {
                required: self.config.frame_bytes,
                provided: buffer.len(),
            });
        }
        let buffer_len = buffer.len();

        let mut dma_buffer = DmaAlignedMut::new(&mut buffer[..self.config.frame_bytes])
            .map_err(ReceiveError::InvalidBuffer)?;

        if wait == CompletionWait::Interrupt {
            self.completion_interrupt = CompletionInterruptReport::default();
            self.check_completion_interrupt_free()?;
        }

        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        dma_buffer.writeback();

        reset_runtime_samples(self);
        reset_runtime_phy_activity(self);

        let dst_start = dma_buffer.as_mut_ptr() as u32;
        self.clear_transfer_status();
        if wait == CompletionWait::Interrupt {
            self.arm_completion_interrupt();
        }
        self.start_transfer(dst_start);
        trace_arm(self, dst_start);
        let dar_at_arm = self.dma_channel.destination_address();
        self.discard_diag.reset_for_capture(dar_at_arm);

        start_source();
        trace_record(self, CaptureTraceEvent::SourceStarted);

        let mut runtime_sampler = RuntimeSampler::new();
        let mut poll_index: u32 = 0;

        let deadline = Instant::now() + self.config.timeout;
        if wait == CompletionWait::Interrupt {
            // The polling loop below then classifies the capture from the
            // registers. After a completion it ends on its first pass.
            self.wait_for_completion_interrupt(deadline);
        }
        let outcome = loop {
            sample_runtime_phy_activity(self);
            let status = self.capture_status();
            maybe_record_runtime_sample(self, &mut runtime_sampler, status);
            trace_poll(self, status);
            poll_index = poll_index.wrapping_add(1);
            if self.discard_diag.armed && poll_index.is_multiple_of(4096) {
                let dar = self.dma_channel.destination_address();
                self.discard_diag.push_dar(dar);
            }

            if status.dma_status.any_error() {
                self.discard_diag.fatal_error_seen = true;
                break ReceiveOutcome::DmaError(status);
            }

            if status.csi_error() {
                let fatal_bridge_error = status.host_errors.any_error()
                    || status.bridge_events.vadr_num_gt
                    || status.bridge_events.vadr_num_lt
                    || status.bridge_events.csi_buf_overrun
                    || status.bridge_events.csi_async_fifo_ovf;
                // Diagnostic mode tolerates only a lone DISCARD. Any real bridge
                // or host error still ends the wait, including when DISCARD is
                // also set. The bit stays set in `int_raw` until the next capture.
                if self.discard_diag.armed && !fatal_bridge_error {
                    if !self.discard_diag.discard_latched {
                        self.discard_diag.discard_count =
                            self.discard_diag.discard_count.saturating_add(1);
                        let dar = self.dma_channel.destination_address();
                        self.discard_diag.push_dar(dar);
                        self.discard_diag.discard_latched = true;
                    }
                } else {
                    self.discard_diag.fatal_error_seen = true;
                    break ReceiveOutcome::CsiError(status);
                }
            } else {
                self.discard_diag.discard_latched = false;
            }

            if status.dma_status.transfer_done() {
                break ReceiveOutcome::Completed(status);
            }

            if Instant::now() > deadline {
                break ReceiveOutcome::Timeout(status);
            }
        };
        trace_record(self, CaptureTraceEvent::WaitEnded);

        if self.discard_diag.armed {
            let status = outcome.status();
            let dar_end = self.dma_channel.destination_address();
            self.discard_diag.dar_end = dar_end;
            self.discard_diag.push_dar(dar_end);
            self.discard_diag.dma_completed = matches!(outcome, ReceiveOutcome::Completed(_));
            self.discard_diag.completed_units = status.dma_status.completed_transfer_units;
        }

        if let Err(reason) = self.stop_transfer() {
            let status_before_shutdown = outcome.status();
            let status_after_shutdown = self.capture_status();
            let abort_in_progress_after_shutdown = self.dma_channel.abort_in_progress();
            let dma_may_still_be_active = status_after_shutdown.dma_status.channel_enabled
                || abort_in_progress_after_shutdown;

            // Buffer ownership can only be returned to safe Rust when the channel is
            // definitely quiesced. If shutdown did not complete and the channel might still be
            // active, abort execution instead of returning a potentially writable buffer.
            if dma_may_still_be_active {
                let debug = self.debug_snapshot();
                let runtime_phy = self.runtime_phy_activity;
                let last_runtime_sample =
                    self.runtime_samples.iter().rev().flatten().next().copied();

                if let Some(last) = last_runtime_sample {
                    panic!(
                        "mipi-csi DMA shutdown failed ({:?}); outcome={} before={{host=0x{:08x},bridge=0x{:08x},dma=0x{:08x},enabled={},units={},fifo_left={}}} after={{host=0x{:08x},bridge=0x{:08x},dma=0x{:08x},enabled={},units={},fifo_left={}}} runtime={{samples={},active_samples={},bridge_fifo_nonzero_seen={},bridge_fifo_max={},clkactivehs_seen={},lane0_left_stopstate_seen={},lane1_left_stopstate_seen={}}} last_sample={{host=0x{:08x},bridge=0x{:08x},dma=0x{:08x},enabled={},units={},fifo_left={}}} regs={{host_phy_stopstate=0x{:08x},host_phy_rx=0x{:08x},host_int_st_main=0x{:08x},bridge_buf_flow_ctl=0x{:08x},bridge_dma_req_cfg=0x{:08x},bridge_int_raw=0x{:08x},bridge_int_st=0x{:08x},bridge_int_ena=0x{:08x},dma_chen0=0x{:08x},dma_chen1=0x{:08x},dma_sar0=0x{:08x},dma_dar0=0x{:08x},dma_block_ts0=0x{:08x},dma_status0=0x{:08x},dma_status1=0x{:08x},dma_intstatus0=0x{:08x},dma_intstatus_enable0=0x{:08x}}} abort_in_progress_after_shutdown={} discard_diag={{armed={},count={},dar_start=0x{:08x},dar_end=0x{:08x}}} channel may still be active",
                        reason,
                        outcome.kind(),
                        status_before_shutdown.host_errors.raw_main_status,
                        status_before_shutdown.bridge_events.raw_status,
                        status_before_shutdown.dma_status.raw_status,
                        status_before_shutdown.dma_status.channel_enabled,
                        status_before_shutdown.dma_status.completed_transfer_units,
                        status_before_shutdown.dma_status.fifo_remaining_units,
                        status_after_shutdown.host_errors.raw_main_status,
                        status_after_shutdown.bridge_events.raw_status,
                        status_after_shutdown.dma_status.raw_status,
                        status_after_shutdown.dma_status.channel_enabled,
                        status_after_shutdown.dma_status.completed_transfer_units,
                        status_after_shutdown.dma_status.fifo_remaining_units,
                        runtime_phy.sample_count,
                        runtime_phy.active_sample_count,
                        runtime_phy.bridge_buf_depth_nonzero_seen,
                        runtime_phy.bridge_buf_depth_max,
                        runtime_phy.clkactivehs_seen,
                        runtime_phy.data_lane0_left_stopstate_seen,
                        runtime_phy.data_lane1_left_stopstate_seen,
                        last.host_errors.raw_main_status,
                        last.bridge_events.raw_status,
                        last.dma_status.raw_status,
                        last.dma_status.channel_enabled,
                        last.dma_status.completed_transfer_units,
                        last.dma_status.fifo_remaining_units,
                        debug.host_phy_stopstate_raw,
                        debug.host_phy_rx_raw,
                        debug.host_int_st_main_raw,
                        debug.bridge_buf_flow_ctl_raw,
                        debug.bridge_dma_req_cfg_raw,
                        debug.bridge_int_raw_raw,
                        debug.bridge_int_st_raw,
                        debug.bridge_int_ena_raw,
                        debug.dma_chen0_raw,
                        debug.dma_chen1_raw,
                        debug.dma_sar0_raw,
                        debug.dma_dar0_raw,
                        debug.dma_block_ts0_raw,
                        debug.dma_status0_raw,
                        debug.dma_status1_raw,
                        debug.dma_intstatus0_raw,
                        debug.dma_intstatus_enable0_raw,
                        abort_in_progress_after_shutdown,
                        self.discard_diag.armed,
                        self.discard_diag.discard_count,
                        self.discard_diag.dar_start,
                        self.discard_diag.dar_end,
                    );
                } else {
                    panic!(
                        "mipi-csi DMA shutdown failed ({:?}); outcome={} before={{host=0x{:08x},bridge=0x{:08x},dma=0x{:08x},enabled={},units={},fifo_left={}}} after={{host=0x{:08x},bridge=0x{:08x},dma=0x{:08x},enabled={},units={},fifo_left={}}} runtime={{samples={},active_samples={},bridge_fifo_nonzero_seen={},bridge_fifo_max={},clkactivehs_seen={},lane0_left_stopstate_seen={},lane1_left_stopstate_seen={}}} regs={{host_phy_stopstate=0x{:08x},host_phy_rx=0x{:08x},host_int_st_main=0x{:08x},bridge_buf_flow_ctl=0x{:08x},bridge_dma_req_cfg=0x{:08x},bridge_int_raw=0x{:08x},bridge_int_st=0x{:08x},bridge_int_ena=0x{:08x},dma_chen0=0x{:08x},dma_chen1=0x{:08x},dma_sar0=0x{:08x},dma_dar0=0x{:08x},dma_block_ts0=0x{:08x},dma_status0=0x{:08x},dma_status1=0x{:08x},dma_intstatus0=0x{:08x},dma_intstatus_enable0=0x{:08x}}} abort_in_progress_after_shutdown={} discard_diag={{armed={},count={},dar_start=0x{:08x},dar_end=0x{:08x}}} no_runtime_sample channel may still be active",
                        reason,
                        outcome.kind(),
                        status_before_shutdown.host_errors.raw_main_status,
                        status_before_shutdown.bridge_events.raw_status,
                        status_before_shutdown.dma_status.raw_status,
                        status_before_shutdown.dma_status.channel_enabled,
                        status_before_shutdown.dma_status.completed_transfer_units,
                        status_before_shutdown.dma_status.fifo_remaining_units,
                        status_after_shutdown.host_errors.raw_main_status,
                        status_after_shutdown.bridge_events.raw_status,
                        status_after_shutdown.dma_status.raw_status,
                        status_after_shutdown.dma_status.channel_enabled,
                        status_after_shutdown.dma_status.completed_transfer_units,
                        status_after_shutdown.dma_status.fifo_remaining_units,
                        runtime_phy.sample_count,
                        runtime_phy.active_sample_count,
                        runtime_phy.bridge_buf_depth_nonzero_seen,
                        runtime_phy.bridge_buf_depth_max,
                        runtime_phy.clkactivehs_seen,
                        runtime_phy.data_lane0_left_stopstate_seen,
                        runtime_phy.data_lane1_left_stopstate_seen,
                        debug.host_phy_stopstate_raw,
                        debug.host_phy_rx_raw,
                        debug.host_int_st_main_raw,
                        debug.bridge_buf_flow_ctl_raw,
                        debug.bridge_dma_req_cfg_raw,
                        debug.bridge_int_raw_raw,
                        debug.bridge_int_st_raw,
                        debug.bridge_int_ena_raw,
                        debug.dma_chen0_raw,
                        debug.dma_chen1_raw,
                        debug.dma_sar0_raw,
                        debug.dma_dar0_raw,
                        debug.dma_block_ts0_raw,
                        debug.dma_status0_raw,
                        debug.dma_status1_raw,
                        debug.dma_intstatus0_raw,
                        debug.dma_intstatus_enable0_raw,
                        abort_in_progress_after_shutdown,
                        self.discard_diag.armed,
                        self.discard_diag.discard_count,
                        self.discard_diag.dar_start,
                        self.discard_diag.dar_end,
                    );
                }
            }

            self.check_destination_bounds(dst_start, buffer_len);

            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            dma_buffer.invalidate();

            return Err(ReceiveError::ShutdownFailed {
                reason,
                status_before_shutdown,
                status_after_shutdown,
            });
        }

        let received_bytes = self.check_destination_bounds(dst_start, buffer_len);

        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        dma_buffer.invalidate();

        match outcome {
            ReceiveOutcome::Completed(_) => Ok(CaptureFrame { received_bytes }),
            ReceiveOutcome::Timeout(status) => Err(ReceiveError::Timeout(status)),
            ReceiveOutcome::DmaError(status) => Err(ReceiveError::DmaError(status)),
            ReceiveOutcome::CsiError(status) => Err(ReceiveError::CsiError(status)),
        }
    }

    fn check_completion_interrupt_free(&self) -> Result<(), ReceiveError> {
        let own = 1 << self.dma_channel.channel_id();
        let enabled_channels = enabled_channels() & !own;
        let asserted_sources = asserted_interrupt_sources() & !own;
        if enabled_channels != 0 || asserted_sources != 0 {
            return Err(ReceiveError::InterruptUnavailable {
                enabled_channels,
                asserted_sources,
            });
        }
        Ok(())
    }

    /// Must run after the channel status was cleared and before the channel is
    /// enabled.
    fn arm_completion_interrupt(&self) {
        COMPLETION_IRQ_ARMED_AT_US.store(now_us(), Ordering::Relaxed);
        COMPLETION_IRQ_STATE.store(COMPLETION_IRQ_ARMED, Ordering::Release);
        register_channel_handler(self.dma_channel, completion_interrupt_handler);
        self.dma_channel
            .set_interrupt_signal_enable(DMA_COMPLETION_INTERRUPT_MASK);
    }

    fn wait_for_completion_interrupt(&mut self, deadline: Instant) {
        while COMPLETION_IRQ_STATE.load(Ordering::Acquire) == COMPLETION_IRQ_ARMED
            && Instant::now() <= deadline
        {
            core::hint::spin_loop();
        }

        self.dma_channel.set_interrupt_signal_enable(0);
        unregister_channel_handler(self.dma_channel);
        // The handler can no longer run, so its results are final.
        let state = COMPLETION_IRQ_STATE.swap(COMPLETION_IRQ_IDLE, Ordering::AcqRel);
        let fired = state != COMPLETION_IRQ_ARMED;
        self.completion_interrupt = CompletionInterruptReport {
            fired,
            own_source: state == COMPLETION_IRQ_OWN,
            channel_status: if fired {
                COMPLETION_IRQ_STATUS.load(Ordering::Relaxed)
            } else {
                0
            },
            latency_from_arm_us: fired.then(|| {
                COMPLETION_IRQ_AT_US
                    .load(Ordering::Relaxed)
                    .wrapping_sub(COMPLETION_IRQ_ARMED_AT_US.load(Ordering::Relaxed))
            }),
        };
    }

    fn start_transfer(&self, dst_addr: u32) {
        self.dma_channel
            .configure_block_transfer(BlockTransferConfig {
                src: BlockTransferEndpointConfig {
                    addr: CSI_BRG_MEM_BASE,
                    ll_pointer_master: LlPointerMasterSelect::Peripheral,
                    addr_inc_mode: AddressIncrementMode::Fixed,
                    burst_size: DW_GDMA_BURST_SIZE_512_ITEMS,
                    axi_burst_len: self.dma_axi.src_burst_len,
                    width: DW_GDMA_WIDTH_64_BITS,
                },
                dst: BlockTransferEndpointConfig {
                    addr: dst_addr,
                    ll_pointer_master: LlPointerMasterSelect::Memory,
                    addr_inc_mode: AddressIncrementMode::Increment,
                    burst_size: DW_GDMA_BURST_SIZE_512_ITEMS,
                    axi_burst_len: self.dma_axi.dst_burst_len,
                    width: DW_GDMA_WIDTH_64_BITS,
                },
                transfer_size_in_units: self.config.transfer_size_units,
                // Match IDF contiguous-transfer path, which relies on DMA
                // transfer-done status instead of block-marker interrupts.
                interrupt_on_block_done: false,
            });

        self.dma_channel.set_enabled(true);
        MIPI_CSI_BRIDGE::regs()
            .csi_en()
            .modify(|_, w| w.csi_brig_en().set_bit());
    }

    /// Stops the capture in the order of TRM §42.7.2: the DMA channel first,
    /// then the bridge.
    fn stop_transfer(&mut self) -> Result<(), ShutdownFailureReason> {
        let result = self.stop_dma_channel();
        MIPI_CSI_BRIDGE::regs()
            .csi_en()
            .modify(|_, w| w.csi_brig_en().clear_bit());
        trace_record(self, CaptureTraceEvent::BridgeDisabled);
        result
    }

    fn stop_dma_channel(&mut self) -> Result<(), ShutdownFailureReason> {
        let channel = self.dma_channel;
        if !channel.is_enabled() {
            return Ok(());
        }

        // TRM §7.5.7.2: the channel halts only after the source reads it has
        // already issued complete. The bridge serves those reads, so it must
        // stay enabled until `CH_EN` reads back clear.
        channel.set_enabled(false);
        if wait_until(DMA_CHANNEL_DISABLE_TIMEOUT_MS, || !channel.is_enabled()) {
            trace_record(self, CaptureTraceEvent::ChannelDisabled);
            return Ok(());
        }

        // TRM §7.5.6: suspend halts the source the same way and reports
        // `CH_SUSPENDED`. Abort is the last resort for a hung channel.
        channel.set_suspended(true);
        wait_until(DMA_CHANNEL_SUSPEND_TIMEOUT_MS, || {
            !channel.is_enabled() || channel.interrupt_status() & DMA_STATUS_CH_SUSPENDED != 0
        });
        channel.trigger_abort();
        if wait_until(DMA_CHANNEL_ABORT_TIMEOUT_MS, || {
            !channel.is_enabled() && !channel.abort_in_progress()
        }) {
            trace_record(self, CaptureTraceEvent::ChannelAborted);
            Ok(())
        } else {
            trace_record(self, CaptureTraceEvent::ShutdownFailed);
            Err(ShutdownFailureReason::AbortTimedOut)
        }
    }

    /// Returns the number of bytes the stopped channel wrote from `dst_start`.
    ///
    /// The bridge is the flow controller and ends the block (TRM §7.5.5), so
    /// `BLOCK_TS` is not known to bound the writes of a source frame that is
    /// larger than configured.
    fn check_destination_bounds(&self, dst_start: u32, buffer_len: usize) -> usize {
        let written = self
            .dma_channel
            .destination_address()
            .saturating_sub(dst_start) as usize;
        if written > buffer_len {
            panic!(
                "mipi-csi DMA wrote {} bytes from 0x{:08x} into a {}-byte buffer",
                written, dst_start, buffer_len
            );
        }
        written
    }

    fn clear_transfer_status(&self) {
        self.dma_channel.clear_all_interrupts();
        self.csi.clear_bridge_interrupts();
    }

    fn capture_status(&self) -> CaptureStatus {
        CaptureStatus {
            host_errors: self.csi.read_host_error_status(),
            bridge_events: self.csi.read_bridge_event_status(),
            dma_status: DmaStatus {
                raw_status: self.dma_channel.interrupt_status(),
                channel_enabled: self.dma_channel.is_enabled(),
                completed_transfer_units: self.dma_channel.completed_transfer_units(),
                fifo_remaining_units: self.dma_channel.fifo_remaining_units(),
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RuntimeSampler {
    next_index: usize,
    next_sample_deadline: Instant,
}

impl RuntimeSampler {
    fn new() -> Self {
        Self {
            next_index: 0,
            next_sample_deadline: Instant::now()
                + Duration::from_millis(CAPTURE_RUNTIME_SAMPLE_INTERVAL_MS),
        }
    }
}

fn reset_runtime_samples(capture: &mut CsiCapture<'_>) {
    capture.runtime_samples = [None; CAPTURE_RUNTIME_SAMPLE_COUNT];
}

fn maybe_record_runtime_sample(
    capture: &mut CsiCapture<'_>,
    sampler: &mut RuntimeSampler,
    status: CaptureStatus,
) {
    if sampler.next_index >= CAPTURE_RUNTIME_SAMPLE_COUNT {
        return;
    }

    if Instant::now() < sampler.next_sample_deadline {
        return;
    }

    capture.runtime_samples[sampler.next_index] = Some(status);
    sampler.next_index += 1;
    sampler.next_sample_deadline =
        Instant::now() + Duration::from_millis(CAPTURE_RUNTIME_SAMPLE_INTERVAL_MS);
}

fn reset_runtime_phy_activity(capture: &mut CsiCapture<'_>) {
    capture.runtime_phy_activity = CapturePhyActivitySummary::empty();
}

fn sample_runtime_phy_activity(capture: &mut CsiCapture<'_>) {
    let host = MIPI_CSI_HOST::regs();
    let bridge = MIPI_CSI_BRIDGE::regs();

    let phy_stopstate = host.phy_stopstate().read();
    let phy_rx = host.phy_rx().read();
    let bridge_buf_depth = bridge.buf_flow_ctl().read().csi_buf_depth().bits();
    let bridge_enabled = bridge.csi_en().read().csi_brig_en().bit_is_set();
    let dma_enabled = capture.dma_channel.is_enabled();

    capture.runtime_phy_activity.record_sample(
        phy_stopstate.bits(),
        phy_rx.bits(),
        bridge_enabled,
        dma_enabled,
        bridge_buf_depth,
        phy_rx.phy_rxclkactivehs().bit_is_set(),
        phy_stopstate.phy_stopstatedata_0().bit_is_set(),
        phy_stopstate.phy_stopstatedata_1().bit_is_set(),
    );
}

fn trace_arm(capture: &mut CsiCapture<'_>, dar_start: u32) {
    let now = Instant::now();
    capture.trace = CaptureTrace::empty();
    capture.trace.dar_start = dar_start;
    capture.trace_recorder = CaptureTraceRecorder {
        armed_at: now,
        last_dar: dar_start,
        last_dar_change: now,
        ..CaptureTraceRecorder::idle()
    };
    trace_record(capture, CaptureTraceEvent::Armed);
    let armed = capture.trace.entries()[0];
    capture.trace_recorder.last_bridge_int_raw = armed.bridge_int_raw;
    capture.trace_recorder.last_dma_intstatus = armed.dma_intstatus;
}

fn trace_record(capture: &mut CsiCapture<'_>, event: CaptureTraceEvent) {
    let bridge = MIPI_CSI_BRIDGE::regs();
    let channel = capture.dma_channel;
    let entry = CaptureTraceEntry {
        event,
        elapsed_us: capture
            .trace_recorder
            .armed_at
            .elapsed()
            .as_micros()
            .min(u32::MAX as u64) as u32,
        dar: channel.destination_address(),
        bridge_int_raw: bridge.int_raw().read().bits(),
        bridge_fifo_depth: bridge.buf_flow_ctl().read().csi_buf_depth().bits(),
        dma_intstatus: channel.interrupt_status(),
        dma_enabled: channel.is_enabled(),
        dma_abort_pending: channel.abort_in_progress(),
        phy_stopstate: MIPI_CSI_HOST::regs().phy_stopstate().read().bits(),
    };
    capture.trace.push(entry);
}

fn trace_poll(capture: &mut CsiCapture<'_>, status: CaptureStatus) {
    capture.trace.polls = capture.trace.polls.saturating_add(1);

    let bridge_raw = status.bridge_events.raw_status;
    if bridge_raw & !capture.trace_recorder.last_bridge_int_raw != 0 {
        capture.trace_recorder.last_bridge_int_raw |= bridge_raw;
        trace_record(capture, CaptureTraceEvent::BridgeEvent);
    }

    let dma_raw = status.dma_status.raw_status;
    if dma_raw & !capture.trace_recorder.last_dma_intstatus != 0 {
        capture.trace_recorder.last_dma_intstatus |= dma_raw;
        trace_record(capture, CaptureTraceEvent::DmaEvent);
    }

    let dar = capture.dma_channel.destination_address();
    let now = Instant::now();
    let recorder = &mut capture.trace_recorder;
    if dar != recorder.last_dar {
        recorder.last_dar = dar;
        recorder.last_dar_change = now;
        if !recorder.first_data_seen {
            recorder.first_data_seen = true;
            trace_record(capture, CaptureTraceEvent::FirstData);
        } else if recorder.stalled {
            recorder.stalled = false;
            trace_record(capture, CaptureTraceEvent::Resumed);
        }
    } else if recorder.first_data_seen
        && !recorder.stalled
        && (now - recorder.last_dar_change).as_micros() >= CAPTURE_TRACE_STALL_US
    {
        recorder.stalled = true;
        trace_record(capture, CaptureTraceEvent::Stalled);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CompletionWait {
    Polling,
    Interrupt,
}

const COMPLETION_IRQ_IDLE: u8 = 0;
const COMPLETION_IRQ_ARMED: u8 = 1;
const COMPLETION_IRQ_OWN: u8 = 2;
const COMPLETION_IRQ_FOREIGN: u8 = 3;

static COMPLETION_IRQ_STATE: AtomicU8 = AtomicU8::new(COMPLETION_IRQ_IDLE);
static COMPLETION_IRQ_STATUS: AtomicU32 = AtomicU32::new(0);
static COMPLETION_IRQ_ARMED_AT_US: AtomicU32 = AtomicU32::new(0);
static COMPLETION_IRQ_AT_US: AtomicU32 = AtomicU32::new(0);

fn now_us() -> u32 {
    Instant::now().duration_since_epoch().as_micros() as u32
}

/// Records the channel status, called by the shared DW-GDMA dispatcher only for
/// the capture channel.
///
/// The status is left latched for the waiting thread, so the source is
/// released by clearing the channel's signal enable, which also makes the
/// handler run at most once per capture.
#[crate::ram]
fn completion_interrupt_handler(channel: DwGdmaChannel, _pending: u32) {
    channel.set_interrupt_signal_enable(0);
    if COMPLETION_IRQ_STATE.load(Ordering::Acquire) != COMPLETION_IRQ_ARMED {
        return;
    }
    let status = channel.interrupt_status();

    COMPLETION_IRQ_STATUS.store(status, Ordering::Relaxed);
    COMPLETION_IRQ_AT_US.store(now_us(), Ordering::Relaxed);
    let state = if status & DMA_COMPLETION_INTERRUPT_MASK != 0 {
        COMPLETION_IRQ_OWN
    } else {
        COMPLETION_IRQ_FOREIGN
    };
    COMPLETION_IRQ_STATE.store(state, Ordering::Release);
}

/// Polls `condition` until it holds or `timeout_ms` passes, then checks it once
/// more.
fn wait_until(timeout_ms: u64, mut condition: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    while Instant::now() <= deadline {
        if condition() {
            return true;
        }
    }
    condition()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReceiveOutcome {
    Completed(CaptureStatus),
    Timeout(CaptureStatus),
    DmaError(CaptureStatus),
    CsiError(CaptureStatus),
}

impl ReceiveOutcome {
    const fn status(self) -> CaptureStatus {
        match self {
            Self::Completed(status)
            | Self::Timeout(status)
            | Self::DmaError(status)
            | Self::CsiError(status) => status,
        }
    }

    const fn kind(self) -> &'static str {
        match self {
            Self::Completed(_) => "completed",
            Self::Timeout(_) => "timeout",
            Self::DmaError(_) => "dma_error",
            Self::CsiError(_) => "csi_error",
        }
    }
}

impl Drop for CsiCapture<'_> {
    fn drop(&mut self) {
        if let Err(reason) = self.stop_transfer() {
            panic!("mipi-csi DMA shutdown failed in Drop: {:?}", reason);
        }
        self.clear_transfer_status();
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

fn phy_set_test_clock(clock_high: bool, clear: bool) {
    MIPI_CSI_HOST::regs().phy_test_ctrl0().write(|w| {
        w.phy_testclk().bit(clock_high);
        w.phy_testclr().bit(clear)
    });
}

fn phy_test_reset() {
    let host = MIPI_CSI_HOST::regs();

    // Same prelude as `mipi_csi_hal_init`: present address 0 with TESTEN
    // held, then pulse `testclr`. Clearing `testclr` while TESTEN is low
    // leaves the D-PHY test interface closed, so the later HS-frequency
    // write does not take effect.
    host.phy_test_ctrl1().write(|w| unsafe {
        w.phy_testen().set_bit();
        w.phy_testdin().bits(0)
    });
    phy_set_test_clock(false, true);
    phy_set_test_clock(false, false);
}

fn phy_write(address: u8, value: u8) {
    let host = MIPI_CSI_HOST::regs();

    phy_set_test_clock(false, false);
    host.phy_test_ctrl1().write(|w| unsafe {
        w.phy_testen().set_bit();
        w.phy_testdin().bits(address)
    });
    phy_set_test_clock(true, false);
    phy_set_test_clock(false, false);

    host.phy_test_ctrl1().write(|w| unsafe {
        w.phy_testen().clear_bit();
        w.phy_testdin().bits(value)
    });
    phy_set_test_clock(true, false);
    phy_set_test_clock(false, false);
}

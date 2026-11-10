#![cfg_attr(docsrs, procmacros::doc_replace)]
//! MIPI CSI-2 receiver controller (ESP32-P4).
//!
//! Configures the CSI host, the CSI bridge and the ISP in bypass, and captures
//! frames into memory through one channel of the shared DW-GDMA (VDMA)
//! controller. Sensor setup, for example over SCCB, is not part of this
//! driver.
//!
//! # Supported configurations
//!
//! - 1 or 2 data lanes, 90 to 1500 Mbps per lane.
//! - RAW8 (data type `0x2a`) only, stored one byte per pixel, unprocessed. The Bayer order is
//!   whatever the sensor sends.
//! - Frame width and height up to 4095 pixels, with a frame size that is a multiple of 8 bytes and
//!   fits one DMA block.
//!
//! Verified on hardware with an SC2336 sensor on the ESP32-P4 Function EV
//! Board: 2 lanes at 288 Mbps, 1024 × 600 RAW8 at 30 fps into PSRAM, alone and
//! while MIPI-DSI streams to a panel. Buffers in internal memory, 1 lane,
//! other lane rates, other resolutions and other sensors use the same code but
//! are not verified.
//!
//! # Power
//!
//! The CSI and DSI D-PHYs share the `VDD_MIPI_DPHY` rail (PMU VO3 at 2.5 V).
//! [`MipiCsi::new`] switches it on if it is off, and the rail stays on until
//! neither this driver nor the MIPI-DSI driver uses it.
//!
//! # Buffers
//!
//! Frame buffers must be DMA-capable and DMA-aligned, in internal memory or
//! PSRAM. The driver does the cache maintenance: it writes the buffer back
//! before the DMA starts and invalidates it before handing it back.
//!
//! # Driver modes
//!
//! [`MipiCsi::with_dma`] returns a [`Blocking`] capture. [`CsiCapture::into_async`] converts it
//! to [`Async`], which adds [`CsiStream::next_frame_async`]. The blocking methods work in both
//! modes, and [`CsiCapture::into_blocking`] converts back.
//!
//! # Capture modes
//!
//! - Single frame: [`CsiCapture::receive`], [`CsiCapture::receive_with`] and
//!   [`CsiCapture::receive_with_interrupt`] borrow the buffer for one frame and block until it
//!   completes, fails or times out. There is no async single-frame capture.
//! - Continuous: [`CsiCapture::into_stream`] takes two `'static` buffers. The DMA interrupt starts
//!   each frame on the other buffer and hands out the newest completed frame as a [`StreamFrame`],
//!   which the DMA does not write while it lives. [`CsiStream::next_frame`] polls for it with a
//!   timeout. [`CsiStream::next_frame_async`] waits for the DMA interrupt instead, and dropping its
//!   future leaves the stream unchanged.
//!
//! # Errors and timeouts
//!
//! The single-frame timeout is [`CaptureConfig::with_timeout`], and [`CsiStream::next_frame`] takes
//! its own. [`CsiStream::next_frame_async`] has no timeout; combine it with a timer to bound the
//! wait. A frame with a DMA error, a CSI host or bridge error, or a size other than the configured
//! one is an error for a single-frame capture. A stream drops such a frame and switches the bridge
//! off and on again, which [`StreamStats`] counts. This includes a frame that the source stopped in
//! the middle of: when the bridge then discards data, the stream stops the DMA channel and drops
//! the frame.
//!
//! # Shutdown
//!
//! The DMA channel is always stopped, with a bounded wait, before a buffer is
//! handed back. If the channel cannot be shown to be stopped, the driver
//! panics instead of returning memory that the DMA can still write. Each
//! capture start drops whatever a stopped capture left in the bridge FIFO.
//!
//! # Limitations
//!
//! - No RAW10, RAW12, YUV422 or RGB formats, and no ISP processing.
//! - One frame per DMA block, so no frames larger than one block.
//! - Embedded data and other non-image packets are not captured.
//! - A source that stops in the middle of a frame leaves the CSI host waiting for that frame's end,
//!   so the host reports a frame boundary or packet error at the next frame start. A single-frame
//!   capture armed right then fails; let the source run for one frame first.
//! - The bridge FIFO flush on every capture start uses the bridge soft reset, whose effect on the
//!   FIFO the TRM does not describe. It was checked on hardware.
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
    marker::PhantomData,
    sync::atomic::{AtomicU8, AtomicU32, Ordering},
};

use procmacros::BuilderLite;
pub use stream::{
    CsiStream,
    IntoStreamError,
    STREAM_BUFFER_COUNT,
    StreamError,
    StreamFrame,
    StreamStartError,
    StreamStats,
};

use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{DmaBufError, aligned::DmaAlignedMut},
    mipi_dphy_power::DphyPower,
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
        channel_interrupt_available,
        enable_controller,
        enabled_channels,
        register_channel_handler,
        unregister_channel_handler,
    },
    peripherals::{HP_SYS_CLKRST, ISP, MIPI_CSI, MIPI_CSI_BRIDGE, MIPI_CSI_HOST},
    private::Sealed,
    system::{GenericPeripheralGuard, Peripheral},
    time::{Duration, Instant},
};

const MIN_LANE_RATE_MBPS: u16 = 90;
const MAX_LANE_RATE_MBPS: u16 = 1500;
// DMA read aperture of the CSI bridge FIFO. The TRM register map does not list
// it; the address is ESP-IDF's `MIPI_CSI_BRG_MEM_BASE`:
// <https://github.com/espressif/esp-idf/blob/v6.0/components/soc/esp32p4/register/hw_ver3/soc/reg_base.h#L169-L170>.
// The next aperture, `0x5010_5000`, is the DSI bridge FIFO.
const CSI_BRG_MEM_BASE: u32 = 0x5010_4000;
const CSI_BRG_AFULL_THRESHOLD: u16 = 960;
const CSI_RAW8_DATA_TYPE: u8 = 0x2A;
// Long-packet data types the bridge passes on: the YUV, RGB and RAW ranges of
// the CSI-2 data type table. ESP-IDF uses the same filter:
// <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_hal_cam/mipi_csi_hal.c#L74-L75>.
const CSI_DATA_TYPE_MIN: u8 = 0x12;
const CSI_DATA_TYPE_MAX: u8 = 0x2F;
// The bridge only receives data through a clocked ISP, so a bypassed ISP still
// gets an 80 MHz clock from PLL_F240M, the CSI input and register updates at
// the next vsync, as in ESP-IDF's processor setup:
// <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_isp/src/isp_core.c#L116-L178>.
const ISP_CLK_SRC_PLL_F240M: u8 = 1;
const ISP_CLK_DIV_80MHZ: u8 = 2;
const ISP_IN_SRC_CSI: u8 = 0;
const ISP_SHADOW_UPDATE_NEXT_VSYNC: u8 = 2;

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

mod stream;

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
        // The bridge is the flow controller and reports the source status from
        // its FIFO aperture, as in ESP-IDF's channel setup:
        // <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L189-L204>.
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
/// [`CaptureConfig`] uses [`Self::DEFAULT`] unless
/// [`CaptureConfig::with_dma_axi_config`] replaces it. Flow control, handshake,
/// addressing, transfer width and burst items are fixed by the driver.
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
    /// Settings of the ESP-IDF v6.0 CSI driver: 5 outstanding requests
    /// ([channel](https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L189-L204)) and 16-beat bursts
    /// ([transfer](https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L495-L511)) on both sides.
    ///
    /// On ESP32-P4 v3.2 with 250 MHz HEX PSRAM, the DMA stalls with these
    /// settings when it writes frames into PSRAM.
    pub const ESP_IDF: Self = Self {
        src_burst_len: DW_GDMA_AXI_BURST_LEN_16,
        dst_burst_len: DW_GDMA_AXI_BURST_LEN_16,
        src_outstanding_requests: DW_GDMA_OUTSTANDING_REQUESTS_5,
        dst_outstanding_requests: DW_GDMA_OUTSTANDING_REQUESTS_5,
    };

    /// [`Self::ESP_IDF`] with one outstanding frame-buffer write, which
    /// captures into PSRAM and internal SRAM.
    pub const DEFAULT: Self = Self {
        dst_outstanding_requests: 1,
        ..Self::ESP_IDF
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

// `PERI_CLK_CTRL03` selector value of the 20 MHz D-PHY configuration clock.
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
    /// AXI burst lengths and outstanding-request limits of the DMA channel.
    dma_axi_config: DmaAxiConfig,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            frame_width: 640,
            frame_height: 480,
            pixel_format: CapturePixelFormat::Raw8,
            timeout: Duration::from_millis(100),
            dma_axi_config: DmaAxiConfig::DEFAULT,
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
    dma_axi: DmaAxiConfig,
}

impl CaptureConfig {
    fn validate(self) -> Result<ValidatedCaptureConfig, CaptureConfigError> {
        if !self.dma_axi_config.is_valid() {
            return Err(CaptureConfigError::InvalidDmaAxiConfig);
        }
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
            dma_axi: self.dma_axi_config,
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
        self.host_errors.any_error() || self.bridge_events.any_error()
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
    /// [`CsiCapture::receive_with_interrupt`] did not arm, because the capture
    /// channel already has a handler in the shared DW-GDMA interrupt
    /// dispatcher. Nothing was started and the buffer was not accessed. The
    /// fields describe the line at that point.
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
                "DW-GDMA interrupt unavailable for the capture channel (enabled channels 0x{enabled_channels:x}, asserted sources 0x{asserted_sources:x})"
            ),
        }
    }
}

impl core::error::Error for ReceiveError {}

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
    /// Returns true when an event that fails a capture is set.
    pub const fn any_error(&self) -> bool {
        self.vadr_num_gt
            || self.vadr_num_lt
            || self.discard
            || self.csi_buf_overrun
            || self.csi_async_fifo_ovf
    }

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
    _dphy_power: DphyPower,
    _csi: MIPI_CSI<'d>,
    _guard: GenericPeripheralGuard<{ Peripheral::MipiCsi as u8 }>,
}

/// CSI capture handle, with one VDMA channel configured for CSI
/// peripheral-to-memory transfers.
///
/// [`MipiCsi::with_dma`] creates it in [`Blocking`] mode. [`Self::into_async`]
/// converts it to [`Async`] mode, where [`CsiStream::next_frame_async`] is
/// available. The blocking methods work in both modes.
pub struct CsiCapture<'d, Dm: DriverMode = Blocking> {
    // Dropped first, so the DMA channel stops before the clocks are gated.
    _shutdown: CaptureShutdown,
    csi: MipiCsi<'d>,
    config: ValidatedCaptureConfig,
    dma_channel: DwGdmaChannel,
    _vdma_guard: GenericPeripheralGuard<{ Peripheral::Vdma as u8 }>,
    _owned_dma_channel: OwnedCsiDmaChannel<'d>,
    completion_interrupt: CompletionInterruptReport,
    _mode: PhantomData<Dm>,
}

/// Stops the capture channel and the ISP bypass when a [`CsiCapture`] is
/// dropped.
struct CaptureShutdown(DwGdmaChannel);

impl Drop for CaptureShutdown {
    fn drop(&mut self) {
        if let Err(reason) = stop_transfer(self.0) {
            panic!("mipi-csi DMA shutdown failed in Drop: {:?}", reason);
        }
        clear_transfer_status(self.0);
        disable_isp_bypass();
    }
}

impl<'d> MipiCsi<'d> {
    /// Initializes MIPI CSI HOST and BRIDGE into an idle state.
    ///
    /// This validates lane rate, enables and resets the peripheral via
    /// `GenericPeripheralGuard`, then applies the minimal verified D-PHY/HOST
    /// sequence for ESP32-P4.
    ///
    /// Powers the `VDD_MIPI_DPHY` rail, which the MIPI-DSI driver shares,
    /// if it is not powered yet. It stays on until neither driver uses it.
    pub fn new(csi: MIPI_CSI<'d>, config: Config) -> Result<Self, ConfigError> {
        if !(MIN_LANE_RATE_MBPS..=MAX_LANE_RATE_MBPS).contains(&config.lane_bit_rate_mbps) {
            return Err(ConfigError::InvalidLaneBitRate);
        }

        let this = Self {
            _dphy_power: DphyPower::acquire(),
            _csi: csi,
            _guard: GenericPeripheralGuard::new(),
        };

        this.init_phy_cfg_clock();
        // The D-PHY test interface is clocked through the bridge
        // (`csi_enableclk`), so the bridge clocks must run before the host
        // setup writes it. ESP-IDF also claims the bridge first:
        // <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L124-L136>.
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
    ) -> Result<CsiCapture<'d, Blocking>, CaptureConfigError> {
        let config = config.validate()?;
        let (owned_dma_channel, channel_id) = take_owned_csi_dma_channel(channel);

        let vdma_guard = GenericPeripheralGuard::new();
        enable_controller();

        let dma_channel = DwGdmaChannel::new(channel_id);
        dma_channel.set_enabled(false);
        dma_channel.configure(csi_dma_channel_config(config.dma_axi));
        dma_channel.set_interrupt_status_enable(DMA_STATUS_ENABLE_MASK);
        dma_channel.set_interrupt_signal_enable(0);
        dma_channel.clear_all_interrupts();

        self.configure_bridge_capture(config);
        clear_bridge_interrupts();
        enable_isp_bypass(config);

        Ok(CsiCapture {
            _shutdown: CaptureShutdown(dma_channel),
            csi: self,
            config,
            dma_channel,
            _vdma_guard: vdma_guard,
            _owned_dma_channel: owned_dma_channel,
            completion_interrupt: CompletionInterruptReport::default(),
            _mode: PhantomData,
        })
    }

    fn init_phy_cfg_clock(&self) {
        // The 20 MHz PLL output gate must be open before any D-PHY
        // test-interface access. The ESP32-P4 clock driver does not open it
        // (`enable_pll_f20m` is a stub), so this driver does.
        HP_SYS_CLKRST::regs()
            .ref_clk_ctrl2()
            .modify(|_, w| w.ref_20m_clk_en().set_bit());

        // Select the 20 MHz D-PHY configuration clock, then pulse its gate low
        // and high, in ESP-IDF's order: <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L128-L136>.
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
        // Input and output formats are equal, so the bridge colour-mode block
        // is enabled and bypassed, as ESP-IDF does for equal formats:
        // <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L594-L601>.
        bridge.host_cm_ctrl().modify(|_, w| {
            w.csi_host_cm_en().set_bit();
            w.csi_host_cm_bypass().set_bit()
        });
        bridge.frame_cfg().modify(|_, w| unsafe {
            w.vadr_num().bits(config.frame_height);
            w.hadr_num().bits(config.frame_width)
        });
        bridge.dma_req_cfg().modify(|_, w| unsafe {
            // The bridge stays the flow controller (reset value) and requests
            // bursts of 512 64-bit words, the DMA burst size of the channel.
            // ESP-IDF sets the same: <https://github.com/espressif/esp-idf/blob/v6.0/components/esp_driver_cam/csi/src/esp_cam_ctlr_csi.c#L185>.
            w.dma_burst_len().bits(512);
            w.dma_cfg_upd_by_blk().clear_bit();
            w.dma_force_rd_status().clear_bit();
            w.csi_dma_flow_controller().set_bit()
        });
        // `int_ena` stays at its reset value of 0: single captures read events
        // from `int_raw`, and only a stream unmasks the discard event.
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

impl<'d> CsiCapture<'d, Blocking> {
    /// Converts the capture to [`Async`] mode.
    ///
    /// [`CsiStream::next_frame_async`] is woken from the shared VDMA
    /// interrupt, which the first VDMA driver to need it binds on its core.
    pub fn into_async(self) -> CsiCapture<'d, Async> {
        self.into_mode()
    }
}

impl<'d> CsiCapture<'d, Async> {
    /// Converts the capture back to [`Blocking`] mode.
    pub fn into_blocking(self) -> CsiCapture<'d, Blocking> {
        self.into_mode()
    }
}

impl<'d, Dm: DriverMode> CsiCapture<'d, Dm> {
    fn into_mode<Dm2: DriverMode>(self) -> CsiCapture<'d, Dm2> {
        CsiCapture {
            _shutdown: self._shutdown,
            csi: self.csi,
            config: self.config,
            dma_channel: self.dma_channel,
            _vdma_guard: self._vdma_guard,
            _owned_dma_channel: self._owned_dma_channel,
            completion_interrupt: self.completion_interrupt,
            _mode: PhantomData,
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
    /// Enable the sensor stream in `start_source`: the DMA and the bridge are
    /// armed by then, so the bridge sees the first frame from its start. The
    /// timeout starts when `start_source` returns.
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
    /// sources of the capture channel, so it can run while other VDMA
    /// channels, such as a MIPI-DSI DPI stream, are active. It returns
    /// [`ReceiveError::InterruptUnavailable`] without starting anything if the
    /// capture channel already has a handler.
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

        let dst_start = dma_buffer.as_mut_ptr() as u32;
        self.clear_transfer_status();
        if wait == CompletionWait::Interrupt {
            self.arm_completion_interrupt();
        }
        self.start_transfer(dst_start);

        start_source();

        let deadline = Instant::now() + self.config.timeout;
        if wait == CompletionWait::Interrupt {
            // The polling loop below then classifies the capture from the
            // registers. After a completion it ends on its first pass.
            self.wait_for_completion_interrupt(deadline);
        }
        let outcome = loop {
            let status = self.capture_status();
            if status.dma_status.any_error() {
                break ReceiveOutcome::DmaError(status);
            }
            if status.csi_error() {
                break ReceiveOutcome::CsiError(status);
            }
            if status.dma_status.transfer_done() {
                break ReceiveOutcome::Completed(status);
            }
            if Instant::now() > deadline {
                break ReceiveOutcome::Timeout(status);
            }
        };

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
                let dma = self.dma_channel;
                panic!(
                    "mipi-csi DMA shutdown failed ({:?}) after {}, the channel may still be active: before={:?} after={:?} abort_in_progress={} dar=0x{:08x} bridge_fifo_words={}",
                    reason,
                    outcome.kind(),
                    status_before_shutdown,
                    status_after_shutdown,
                    abort_in_progress_after_shutdown,
                    dma.destination_address(),
                    MIPI_CSI_BRIDGE::regs()
                        .buf_flow_ctl()
                        .read()
                        .csi_buf_depth()
                        .bits(),
                );
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
        if !channel_interrupt_available(self.dma_channel) {
            let own = 1 << self.dma_channel.channel_id();
            return Err(ReceiveError::InterruptUnavailable {
                enabled_channels: enabled_channels() & !own,
                asserted_sources: asserted_interrupt_sources() & !own,
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
        flush_bridge_fifo();
        self.dma_channel
            .configure_block_transfer(self.block_transfer(dst_addr));

        self.dma_channel.set_enabled(true);
        MIPI_CSI_BRIDGE::regs()
            .csi_en()
            .modify(|_, w| w.csi_brig_en().set_bit());
    }

    fn block_transfer(&self, dst_addr: u32) -> BlockTransferConfig {
        BlockTransferConfig {
            src: BlockTransferEndpointConfig {
                addr: CSI_BRG_MEM_BASE,
                ll_pointer_master: LlPointerMasterSelect::Peripheral,
                addr_inc_mode: AddressIncrementMode::Fixed,
                burst_size: DW_GDMA_BURST_SIZE_512_ITEMS,
                axi_burst_len: self.config.dma_axi.src_burst_len,
                width: DW_GDMA_WIDTH_64_BITS,
            },
            dst: BlockTransferEndpointConfig {
                addr: dst_addr,
                ll_pointer_master: LlPointerMasterSelect::Memory,
                addr_inc_mode: AddressIncrementMode::Increment,
                burst_size: DW_GDMA_BURST_SIZE_512_ITEMS,
                axi_burst_len: self.config.dma_axi.dst_burst_len,
                width: DW_GDMA_WIDTH_64_BITS,
            },
            transfer_size_in_units: self.config.transfer_size_units,
            // One contiguous block per frame. The transfer-done status ends
            // it, so no block-done interrupt is needed.
            interrupt_on_block_done: false,
        }
    }

    fn stop_transfer(&mut self) -> Result<(), ShutdownFailureReason> {
        stop_transfer(self.dma_channel)
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
        clear_transfer_status(self.dma_channel);
    }

    fn capture_status(&self) -> CaptureStatus {
        CaptureStatus {
            host_errors: host_error_status(),
            bridge_events: bridge_event_status(),
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

/// Stops the capture in the order of TRM §42.7.2: the DMA channel first,
/// then the bridge.
fn stop_transfer(channel: DwGdmaChannel) -> Result<(), ShutdownFailureReason> {
    let result = stop_dma_channel(channel);
    MIPI_CSI_BRIDGE::regs()
        .csi_en()
        .modify(|_, w| w.csi_brig_en().clear_bit());
    result
}

fn stop_dma_channel(channel: DwGdmaChannel) -> Result<(), ShutdownFailureReason> {
    if !channel.is_enabled() {
        return Ok(());
    }

    // TRM §7.5.7.2: the channel halts only after the source reads it has
    // already issued complete. The bridge serves those reads, so it must
    // stay enabled until `CH_EN` reads back clear.
    channel.set_enabled(false);
    if wait_until(DMA_CHANNEL_DISABLE_TIMEOUT_MS, || !channel.is_enabled()) {
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
        Ok(())
    } else {
        Err(ShutdownFailureReason::AbortTimedOut)
    }
}

fn clear_transfer_status(channel: DwGdmaChannel) {
    channel.clear_all_interrupts();
    clear_bridge_interrupts();
    clear_host_errors();
}

/// Drops the host errors latched since the last read, for example the frame
/// boundary error left by a source that stopped mid-frame. `INT_ST_MAIN`
/// clears on read.
fn clear_host_errors() {
    MIPI_CSI_HOST::regs().int_st_main().read();
}

/// Clears the bridge event bits. Only called while the bridge is disabled.
fn clear_bridge_interrupts() {
    unsafe {
        MIPI_CSI_BRIDGE::regs()
            .int_clr()
            .write(|w| w.bits(CSI_BRIDGE_INTERRUPT_MASK))
    };
}

/// Drops what the bridge FIFO still holds of a frame that a stop cut short.
///
/// Disabling the bridge keeps its FIFO, so without this the next capture
/// starts with the tail of the old frame. After a mid-frame stop the bridge
/// could then stop requesting DMA bursts altogether, with more than a burst
/// buffered. The soft reset empties the FIFO and keeps the bridge
/// configuration (checked on hardware for every configuration register).
///
/// Only call this with the bridge disabled and the capture channel idle.
pub(super) fn flush_bridge_fifo() {
    let bridge = MIPI_CSI_BRIDGE::regs();
    debug_assert!(
        !bridge.csi_en().read().csi_brig_en().bit(),
        "the CSI bridge FIFO is only flushed with the bridge disabled"
    );
    bridge.csi_en().modify(|_, w| w.csi_brig_rst().set_bit());
    bridge.csi_en().modify(|_, w| w.csi_brig_rst().clear_bit());
}

/// Passes the CSI input through the ISP unprocessed. The bridge receives no
/// data while the ISP clock or its MIPI input gate is off.
fn enable_isp_bypass(config: ValidatedCaptureConfig) {
    let clkrst = HP_SYS_CLKRST::regs();
    clkrst.peri_clk_ctrl25().modify(|_, w| unsafe {
        w.isp_clk_src_sel().bits(ISP_CLK_SRC_PLL_F240M);
        w.isp_clk_en().set_bit()
    });
    clkrst
        .peri_clk_ctrl26()
        .modify(|_, w| unsafe { w.isp_clk_div_num().bits(ISP_CLK_DIV_80MHZ) });
    clkrst.hp_rst_en0().modify(|_, w| w.rst_en_isp().set_bit());
    clkrst
        .hp_rst_en0()
        .modify(|_, w| w.rst_en_isp().clear_bit());

    let isp = ISP::regs();
    isp.cntl().write(|w| unsafe { w.bits(0) });
    isp.int_ena().write(|w| unsafe { w.bits(0) });
    isp.int_clr().write(|w| unsafe { w.bits(u32::MAX) });
    // The reset values enable the processing stages and expect line start and
    // line end packets, so every field is written from zero.
    isp.clk_en()
        .write(|w| unsafe { w.bits(0) }.clk_en().set_bit());
    isp.cntl().write(|w| unsafe {
        w.bits(0);
        w.isp_in_src().bits(ISP_IN_SRC_CSI);
        w.mipi_data_en().set_bit()
    });
    // In bypass the line length is counted in 32-bit words.
    let line_words = (config.frame_width as u32 * 8).div_ceil(32) as u16;
    isp.frame_cfg().write(|w| unsafe {
        w.bits(0);
        w.vadr_num().bits(config.frame_height - 1);
        w.hadr_num().bits(line_words - 1)
    });
    isp.shadow_reg_ctrl()
        .modify(|_, w| unsafe { w.shadow_update_sel().bits(ISP_SHADOW_UPDATE_NEXT_VSYNC) });
}

fn disable_isp_bypass() {
    let isp = ISP::regs();
    isp.cntl().write(|w| unsafe { w.bits(0) });
    isp.clk_en().write(|w| w.clk_en().clear_bit());
    HP_SYS_CLKRST::regs()
        .peri_clk_ctrl25()
        .modify(|_, w| w.isp_clk_en().clear_bit());
}

impl Drop for MipiCsi<'_> {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn host_error_status() -> HostErrorStatus {
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

fn bridge_event_status() -> BridgeEventStatus {
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

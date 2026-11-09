//! Capture one frame from SC2336 over MIPI-CSI on ESP32-P4 Function EV Board.
//!
//! Uses the official Espressif SC2336 mode table:
//! `MIPI_2lane_24Minput_RAW8_1024x600_30fps`.
//!
//! The ISP, the DMA channel and the CSI bridge are armed before the sensor
//! stream starts, in the order used by Espressif's esp-video framework.
//!
//! Selected at build time:
//! - `CSI_CAPTURE_MODE=A` (default): ISP bypass, RAW8 frame into PSRAM.
//! - `CSI_CAPTURE_MODE=B`: ISP RAW8 to RGB565 into PSRAM, with the RAW8-sized DMA block that the
//!   ESP-IDF CSI driver programs for this configuration.
//! - `CSI_TEST_PATTERN=1`: SC2336 test pattern instead of the image.
//! - `CSI_HOST_ERRORS=1`: unmask the CSI host error interrupts, so that host errors end the
//!   capture.
//! - `CSI_DMA_PROFILE`: AXI settings of the CSI DMA channel. `dst-osr-1` (default) allows one
//!   outstanding frame-buffer write, `dst-awlen-4` uses 4-beat frame-buffer writes, `src-osr-1`
//!   allows one outstanding bridge read, and `idf` keeps the ESP-IDF driver settings. The first
//!   three passed `vdma-mem2mem-psram` into 250 MHz PSRAM, where the `idf` settings stalled.
//! - `CSI_FRAME_EXPORT=1`: after a valid mode A frame, print it as `@@SC2336FRAME` text lines for
//!   `examples/qa/tools/csi_frame_extract.py`. About 860 KB of console output.
//! - `CSI_EXPOSURE_LINES=1..=994`: exposure in rows of 33.333 us, instead of the mode default of
//!   894 (`0x3e00..=0x3e02`).
//! - `CSI_GAIN_INDEX=0..=193`: entry of the official SC2336 gain map (`0x3e06`, `0x3e07`,
//!   `0x3e09`); 0 is 1x, 193 is the driver's default 66x limit. Unset leaves the reset value.
//! - `CSI_REPEAT_FRAMES=1..=1000`: mode A only, capture this many frames one after another into the
//!   same buffer and print one summary line per frame. With `CSI_FRAME_EXPORT=1`, the last frame is
//!   exported.
//! - `CSI_SENSOR_CONTINUOUS=1`: with `CSI_REPEAT_FRAMES`, start the SC2336 stream once before the
//!   first frame and stop it after the last, instead of around every frame. Each frame is still one
//!   `receive_with`/`receive_with_interrupt` call that arms the DMA and the bridge while the sensor
//!   streams.
//! - `CSI_STREAM=1`: with `CSI_REPEAT_FRAMES`, capture with `CsiCapture::into_stream` into two
//!   PSRAM buffers that the DMA interrupt handler alternates between, with the sensor streaming
//!   throughout, and take this many frames with `CsiStream::next_frame`. Each buffer is followed by
//!   a guard region, and both start filled with a sentinel that the first frame in each buffer must
//!   fully overwrite.
//! - `CSI_STREAM_FAULTS=1`: with `CSI_STREAM`, after the stream run, check a `next_frame` timeout
//!   with the sensor stopped, a consumer holding a frame while the sensor streams, and repeated
//!   stop/start at varying points of the frame. Prints `restart-snap` lines with the sensor, D-PHY,
//!   host, bridge, ISP and DMA state around the sensor restart.
//! - `CSI_ASYNC=1`: with `CSI_STREAM`, convert the capture with `into_async` and take the frames
//!   with `CsiStream::next_frame_async`, polled again only after its waker fired. Then drop a
//!   pending `next_frame_async` future, check that frames still arrive, stop and restart the
//!   stream, and capture one frame after `into_blocking`, with the sensor already streaming because
//!   the sensor stop can cut a frame. `CSI_ASYNC_SENSOR_RESTART=1` also stops and restarts the
//!   sensor while the stream stays armed, which reproduces a known stall.
//! - `CSI_FAULT_ROUNDS=<n>`: with `CSI_STREAM`, repeat the stream run (and the fault cases) `n`
//!   times in one boot.
//! - `CSI_QUIET=1`: with `CSI_REPEAT_FRAMES`, print the per-frame line only for failures and every
//!   100th frame, so the console does not slow the capture loop.
//! - `CSI_SKIP_ISP=1`: ISP bypass modes only, leave the ISP clock, reset and input gate untouched,
//!   to check whether the bypass path needs them.
//! - `CSI_COMPLETION`: `poll` (default) waits with `receive_with`, `interrupt` with
//!   `receive_with_interrupt`. In interrupt mode a frame also fails if the channel's own DW-GDMA
//!   completion interrupt did not end the wait.
//! - `CSI_DSI_COEXIST=1`: with `CSI_REPEAT_FRAMES`, first start the `lcd-mipi-dsi` colour wheel on
//!   the EK79007 panel (MIPI-DSI DPI on `VDMA_CH0`, LCD reset GPIO27, backlight GPIO26), then
//!   capture on `VDMA_CH1` while it streams. A frame also fails if the DSI block-done counter did
//!   not advance during its capture. At the end CSI and then DSI are dropped and their channels
//!   checked.
//! - `CSI_DPHY_LIFECYCLE=1`: instead of the capture run, check the `VDD_MIPI_DPHY` rail that CSI
//!   and DSI share: CSI alone, DSI alone, both in either creation and drop order, then the combined
//!   cases again. After every step the PMU rail must be on exactly while a driver lives, a live CSI
//!   driver must capture a full RAW8 frame, and a live DSI driver must keep its PHY PLL locked.
//!   Prints `DPHY LIFECYCLE RESULT: PASS` or `FAIL`.
//!
//! Every run ends with one `QA_RESULT: PASS test=<name> ...` or
//! `QA_RESULT: FAIL test=<name> ...` line, printed after all checks, for a
//! host script to stop the monitor on.
//% CHIP_FILTER: mipi_csi_driver_supported && mipi_dsi_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{format, string::String, vec::Vec};
use core::{
    alloc::Layout,
    future::Future,
    pin::pin,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    clock::ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    delay::Delay,
    dma::aligned::DmaAlignedMut,
    gpio::{Level, Output, OutputConfig},
    i2c::{
        self,
        master::{Config as I2cConfig, I2c},
    },
    main,
    mipi_csi::{
        CaptureConfig,
        CaptureFrame,
        CapturePixelFormat,
        CaptureStatus,
        Config,
        CsiCapture,
        DataLanes,
        DmaAxiConfig,
        MipiCsi,
        ReceiveError,
        StreamError,
    },
    mipi_dsi::{
        Config as DsiConfig,
        DataLanes as DsiDataLanes,
        MipiDsi,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, DsiDpi, FrameTiming},
    },
    peripherals::{
        HP_SYS_CLKRST,
        INTERRUPT_CORE0,
        INTERRUPT_CORE1,
        MIPI_CSI,
        MIPI_CSI_BRIDGE,
        MIPI_CSI_HOST,
        MIPI_DSI,
        MIPI_DSI_BRIDGE,
        MIPI_DSI_HOST,
        PMU,
        VDMA,
        VDMA_CH0,
        VDMA_CH1,
    },
    psram,
    time::{Duration, Rate},
};
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

const SC2336_SCCB_ADDR: u8 = 0x30;
const SC2336_REG_SENSOR_ID_H: u16 = 0x3107;
const SC2336_REG_SENSOR_ID_L: u16 = 0x3108;
const SC2336_EXPECTED_PID: u16 = 0xCB3A;
const SC2336_REG_STREAM_CTRL: u16 = 0x0100;
// `sc2336_set_test_pattern` in esp-video-components `sc2336.c`.
const SC2336_REG_TEST_PATTERN: u16 = 0x4501;
const SC2336_TEST_PATTERN_BIT: u8 = 1 << 3;

const SC2336_FRAME_WIDTH: u16 = 1024;
const SC2336_FRAME_HEIGHT: u16 = 600;
const SC2336_FRAME_BYTES: usize = SC2336_FRAME_WIDTH as usize * SC2336_FRAME_HEIGHT as usize;
const RGB565_BYTES_PER_PIXEL: usize = 2;
const RGB565_FRAME_BYTES: usize = SC2336_FRAME_BYTES * RGB565_BYTES_PER_PIXEL;
const SC2336_FRAME_TRANSFER_UNITS: u32 = (SC2336_FRAME_BYTES / 8) as u32;
const SC2336_LANES: DataLanes = DataLanes::_2;

// esp-video-components (SC2336 mode table): mipi_info.mipi_clk = 288_000_000.
// That field is documented as clock-lane frequency in Hz, while this specific
// mode list and register-list name also label "288Mbps". We keep the first-pass
// lane-rate configuration aligned with Espressif's mode table value.
const SC2336_MIPI_CLK_HZ: u32 = 288_000_000;
const SC2336_LANE_BIT_RATE_MBPS: u16 = (SC2336_MIPI_CLK_HZ / 1_000_000) as u16;
const SC2336_DDR_EQUIVALENT_MBPS: u16 = (SC2336_MIPI_CLK_HZ / 500_000) as u16;
// IDF CSI controller default packet filter for raw/yuv/rgb paths.
const CSI_IDF_DEFAULT_DATA_TYPE_MIN: u8 = 0x12;
const CSI_IDF_DEFAULT_DATA_TYPE_MAX: u8 = 0x2F;
// IDF ISP control registers.
const ISP_CLK_EN_REG_ADDR: usize = 0x500A_1004;
const ISP_CNTL_REG_ADDR: usize = 0x500A_1008;
const ISP_FRAME_CFG_REG_ADDR: usize = 0x500A_1010;
const ISP_CLK_EN_BIT: u32 = 1 << 0;
const ISP_CNTL_MIPI_DATA_EN_BIT: u32 = 1 << 0;
const ISP_CNTL_ISP_EN_BIT: u32 = 1 << 1;
const ISP_CNTL_DEMOSAIC_EN_BIT: u32 = 1 << 6;
const ISP_CNTL_RGB2YUV_EN_BIT: u32 = 1 << 10;
const ISP_CNTL_YUV2RGB_EN_BIT: u32 = 1 << 13;
const ISP_CNTL_OUT_TYPE_SHIFT: u32 = 29;
const ISP_CNTL_OUT_TYPE_MASK: u32 = 0b111 << ISP_CNTL_OUT_TYPE_SHIFT;
const ISP_CNTL_OUT_TYPE_RGB565: u32 = 4 << ISP_CNTL_OUT_TYPE_SHIFT;
const ISP_CNTL_ISP_DATA_TYPE_SHIFT: u32 = 25;
const ISP_CNTL_ISP_DATA_TYPE_MASK: u32 = 0b11 << ISP_CNTL_ISP_DATA_TYPE_SHIFT;
const ISP_CNTL_ISP_DATA_TYPE_RAW8: u32 = 0 << ISP_CNTL_ISP_DATA_TYPE_SHIFT;
const ISP_CNTL_ISP_IN_SRC_SHIFT: u32 = 27;
const ISP_CNTL_ISP_IN_SRC_MASK: u32 = 0b11 << ISP_CNTL_ISP_IN_SRC_SHIFT;
const ISP_CNTL_ISP_IN_SRC_CSI: u32 = 0 << ISP_CNTL_ISP_IN_SRC_SHIFT;
const ISP_FRAME_CFG_VADR_NUM_MASK: u32 = 0x0fff;
const ISP_FRAME_CFG_HADR_NUM_SHIFT: u32 = 12;
const ISP_FRAME_CFG_HADR_NUM_MASK: u32 = 0x0fff << ISP_FRAME_CFG_HADR_NUM_SHIFT;
const ISP_FRAME_CFG_HSYNC_START_EXIST_BIT: u32 = 1 << 29;
const ISP_FRAME_CFG_HSYNC_END_EXIST_BIT: u32 = 1 << 30;
const ISP_INT_RAW_REG_ADDR: usize = 0x500A_1064;
const ISP_INT_ENA_REG_ADDR: usize = 0x500A_106C;
const ISP_INT_CLR_REG_ADDR: usize = 0x500A_1070;
const ISP_SHADOW_REG_CTRL_REG_ADDR: usize = 0x500A_1270;
const ISP_SHADOW_UPDATE_SEL_SHIFT: u32 = 30;
const ISP_SHADOW_UPDATE_SEL_MASK: u32 = 0b11 << ISP_SHADOW_UPDATE_SEL_SHIFT;
const ISP_SHADOW_UPDATE_ONLY_NEXT_VSYNC: u32 = 2 << ISP_SHADOW_UPDATE_SEL_SHIFT;
// `ISP_LL_EVENT_ERROR_MASK` in ESP-IDF `isp_ll.h`.
const ISP_INT_ERROR_MASK: u32 = 0x8000_003F;
const ISP_INT_NAMES: &[(u32, &str)] = &[
    (0, "data_type_err"),
    (1, "async_fifo_ovf"),
    (2, "buf_full"),
    (3, "hvnum_setting_err"),
    (4, "data_type_setting_err"),
    (5, "mipi_hnum_unmatch"),
    (14, "frame"),
    (19, "demosaic_frame"),
    (23, "rgb2yuv_frame"),
    (26, "yuv2rgb_frame"),
    (27, "tail_idi_frame"),
    (28, "header_idi_frame"),
    (31, "crop_err"),
];
const BRIDGE_INT_NAMES: &[(u32, &str)] = &[
    (0, "vadr_num_gt"),
    (1, "vadr_num_lt"),
    (2, "discard"),
    (3, "buf_overrun"),
    (4, "async_fifo_ovf"),
    (5, "dma_cfg_has_updated"),
];
// Bits 5 to 14 and 16 to 21 of `CH_INTSTATUS0`.
const BRIDGE_DMA_BURST_BYTES: usize = 512 * 8;
const HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_EN_BIT: u32 = 1 << 25;
const HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_SHIFT: u32 = 23;
const HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_MASK: u32 =
    0b11 << HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_SHIFT;
const HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_PLL240: u32 =
    0b01 << HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_SHIFT;
const HP_SYS_CLKRST_PERI_CLK_CTRL26_ISP_CLK_DIV_NUM_MASK: u32 = 0xff;
const HP_SYS_CLKRST_PERI_CLK_CTRL26_ISP_CLK_DIV_NUM_80MHZ_FROM_240MHZ: u32 = 2;
const HP_SYS_CLKRST_HP_RST_EN0_RST_EN_ISP_BIT: u32 = 1 << 29;

// PMU VO3 (ext_ldo_p0_0p2a) configured to 2.5 V for VDD_MIPI_DPHY.
const MIPI_DPHY_LDO_CTRL: u32 = 0x4020_0180;
const MIPI_DPHY_LDO_ANA_2P5V: u32 = (9u32 << 28) | (6u32 << 23);

// Espressif reference register list:
// sc2336_mipi_2lane_24Minput_1024x600_raw8_30fps.
const SC2336_MODE_1024X600_RAW8_30FPS: &[(u16, u8)] = &[
    (0x0103, 0x01),
    (0x0100, 0x00),
    (0x36e9, 0x80),
    (0x37f9, 0x80),
    (0x301f, 0xc7),
    (0x3031, 0x08),
    (0x3037, 0x00),
    (0x3106, 0x05),
    (0x3200, 0x01),
    (0x3201, 0xb4),
    (0x3202, 0x00),
    (0x3203, 0xf0),
    (0x3204, 0x05),
    (0x3205, 0xd3),
    (0x3206, 0x03),
    (0x3207, 0x4f),
    (0x3208, 0x04),
    (0x3209, 0x00),
    (0x320a, 0x02),
    (0x320b, 0x58),
    (0x320c, 0x09),
    (0x320d, 0x60),
    (0x320e, 0x03),
    (0x320f, 0xe8),
    (0x3210, 0x00),
    (0x3211, 0x10),
    (0x3212, 0x00),
    (0x3213, 0x04),
    (0x3248, 0x04),
    (0x3249, 0x0b),
    (0x3253, 0x08),
    (0x3301, 0x09),
    (0x3302, 0xff),
    (0x3303, 0x10),
    (0x3306, 0x60),
    (0x3307, 0x02),
    (0x330a, 0x01),
    (0x330b, 0x10),
    (0x330c, 0x16),
    (0x330d, 0xff),
    (0x3318, 0x02),
    (0x3321, 0x0a),
    (0x3327, 0x0e),
    (0x332b, 0x12),
    (0x3333, 0x10),
    (0x3334, 0x40),
    (0x335e, 0x06),
    (0x335f, 0x0a),
    (0x3364, 0x1f),
    (0x337c, 0x02),
    (0x337d, 0x0e),
    (0x3390, 0x09),
    (0x3391, 0x0f),
    (0x3392, 0x1f),
    (0x3393, 0x20),
    (0x3394, 0x20),
    (0x3395, 0xff),
    (0x33a2, 0x04),
    (0x33b1, 0x80),
    (0x33b2, 0x68),
    (0x33b3, 0x42),
    (0x33f9, 0x78),
    (0x33fb, 0xd8),
    (0x33fc, 0x0f),
    (0x33fd, 0x1f),
    (0x349f, 0x03),
    (0x34a6, 0x0f),
    (0x34a7, 0x1f),
    (0x34a8, 0x42),
    (0x34a9, 0x06),
    (0x34aa, 0x01),
    (0x34ab, 0x28),
    (0x34ac, 0x01),
    (0x34ad, 0x90),
    (0x3630, 0xf4),
    (0x3633, 0x22),
    (0x3639, 0xf4),
    (0x363c, 0x47),
    (0x3670, 0x09),
    (0x3674, 0xf4),
    (0x3675, 0xfb),
    (0x3676, 0xed),
    (0x367c, 0x09),
    (0x367d, 0x0f),
    (0x3690, 0x22),
    (0x3691, 0x22),
    (0x3692, 0x22),
    (0x3698, 0x89),
    (0x3699, 0x96),
    (0x369a, 0xd0),
    (0x369b, 0xd0),
    (0x369c, 0x09),
    (0x369d, 0x0f),
    (0x36a2, 0x09),
    (0x36a3, 0x0f),
    (0x36a4, 0x1f),
    (0x36d0, 0x01),
    (0x36ea, 0x08),
    (0x36eb, 0x0a),
    (0x36ec, 0x1a),
    (0x36ed, 0x18),
    (0x3722, 0xe1),
    (0x3724, 0x41),
    (0x3725, 0xc1),
    (0x3728, 0x20),
    (0x37fa, 0x08),
    (0x37fb, 0x32),
    (0x37fc, 0x11),
    (0x37fd, 0x37),
    (0x3900, 0x0d),
    (0x3905, 0x98),
    (0x391b, 0x81),
    (0x391c, 0x10),
    (0x3933, 0x81),
    (0x3934, 0xc5),
    (0x3940, 0x68),
    (0x3941, 0x00),
    (0x3942, 0x01),
    (0x3943, 0xc6),
    (0x3952, 0x02),
    (0x3953, 0x0f),
    (0x3e01, 0x37),
    (0x3e02, 0xe0),
    (0x3e08, 0x1f),
    (0x3e1b, 0x14),
    (0x4509, 0x38),
    (0x4819, 0x05),
    (0x481b, 0x03),
    (0x481d, 0x0a),
    (0x481f, 0x02),
    (0x4821, 0x08),
    (0x4823, 0x03),
    (0x4825, 0x02),
    (0x4827, 0x03),
    (0x4829, 0x04),
    (0x5799, 0x06),
    (0x5ae0, 0xfe),
    (0x5ae1, 0x40),
    (0x5ae2, 0x30),
    (0x5ae3, 0x28),
    (0x5ae4, 0x20),
    (0x5ae5, 0x30),
    (0x5ae6, 0x28),
    (0x5ae7, 0x20),
    (0x5ae8, 0x3c),
    (0x5ae9, 0x30),
    (0x5aea, 0x28),
    (0x5aeb, 0x3c),
    (0x5aec, 0x30),
    (0x5aed, 0x28),
    (0x5aee, 0xfe),
    (0x5aef, 0x40),
    (0x5af4, 0x30),
    (0x5af5, 0x28),
    (0x5af6, 0x20),
    (0x5af7, 0x30),
    (0x5af8, 0x28),
    (0x5af9, 0x20),
    (0x5afa, 0x3c),
    (0x5afb, 0x30),
    (0x5afc, 0x28),
    (0x5afd, 0x3c),
    (0x5afe, 0x30),
    (0x5aff, 0x28),
    (0x36e9, 0x53),
    (0x37f9, 0x53),
];

const SC2336_KEY_MODE_REGS: &[u16] = &[
    0x0100, // stream control
    0x301f, // sensor output control
    0x3031, // output format related
    0x3208, // output width high
    0x3209, // output width low
    0x320a, // output height high
    0x320b, // output height low
    0x320e, // frame length high
    0x320f, // frame length low
    0x4819, // mipi timing
    0x481b, // mipi timing
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CaptureMode {
    /// ISP bypass, RAW8 frame into PSRAM.
    Raw8Psram,
    /// ISP RAW8 to RGB565 into PSRAM, `BLOCK_TS` sized for RAW8 like ESP-IDF.
    Rgb565Psram,
}

impl CaptureMode {
    fn selected() -> Self {
        match option_env!("CSI_CAPTURE_MODE") {
            Some("B") | Some("b") => Self::Rgb565Psram,
            _ => Self::Raw8Psram,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Raw8Psram => "A-raw8-bypass-psram",
            Self::Rgb565Psram => "B-rgb565-isp-psram",
        }
    }

    const fn isp_enabled(self) -> bool {
        matches!(self, Self::Rgb565Psram)
    }

    const fn bytes_per_pixel(self) -> usize {
        if self.isp_enabled() {
            RGB565_BYTES_PER_PIXEL
        } else {
            1
        }
    }

    /// Bytes the capture should end with.
    const fn expected_bytes(self) -> usize {
        match self {
            Self::Raw8Psram => SC2336_FRAME_BYTES,
            Self::Rgb565Psram => RGB565_FRAME_BYTES,
        }
    }

    /// `BLOCK_TS` of the capture, in bytes. The driver sizes it for RAW8.
    const fn dma_block_bytes(self) -> usize {
        SC2336_FRAME_BYTES
    }
}

fn env_flag(value: Option<&str>) -> bool {
    value == Some("1")
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Completion {
    Polling,
    Interrupt,
}

impl Completion {
    fn selected() -> Self {
        match option_env!("CSI_COMPLETION") {
            None | Some("poll") => Self::Polling,
            Some("interrupt") => Self::Interrupt,
            Some(other) => panic!("unknown CSI_COMPLETION '{other}', use poll or interrupt"),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Polling => "poll",
            Self::Interrupt => "interrupt",
        }
    }
}

fn receive_frame(
    capture: &mut CsiCapture<'_>,
    completion: Completion,
    buffer: &mut [u8],
    start_source: impl FnOnce(),
) -> Result<CaptureFrame, ReceiveError> {
    match completion {
        Completion::Polling => capture.receive_with(buffer, start_source),
        Completion::Interrupt => capture.receive_with_interrupt(buffer, start_source),
    }
}

/// In interrupt mode, a frame only counts if the capture channel's own
/// completion interrupt ended the wait, not the timeout or the polling fallback.
fn completion_interrupt_ok(
    label: &str,
    capture: &CsiCapture<'_>,
    completion: Completion,
    verbose: bool,
) -> bool {
    if completion == Completion::Polling {
        return true;
    }
    let report = capture.completion_interrupt_report();
    let ok = report.fired && report.own_source;
    if verbose || !ok {
        println!(
            "{label}: completion_irq fired={} own_source={} latency_from_arm={:?}us channel_status={:#x}",
            report.fired, report.own_source, report.latency_from_arm_us, report.channel_status
        );
    }
    if !ok {
        println!(
            "{label}: completion_irq FAIL, {}",
            if !report.fired {
                "the wait ended at the timeout without the interrupt"
            } else {
                "another source drove the DW-GDMA line, the wait fell back to polling"
            }
        );
    }
    ok
}

const LCD_H_ACTIVE: u32 = 1024;
const LCD_V_ACTIVE: u32 = 600;
const LCD_FB_BYTES: usize = LCD_H_ACTIVE as usize * LCD_V_ACTIVE as usize * 2;
/// About six frames at the panel's 58.8 Hz.
const LCD_VSYNC_TIMEOUT_MS: u64 = 100;
const LCD_BASELINE_STEPS: u32 = 60;
const LCD_TAIL_MS: u64 = 500;

/// `CSI_DSI_COEXIST=1`: the EK79007 panel and colour wheel of `lcd-mipi-dsi`,
/// streaming from PSRAM through MIPI-DSI DPI on `VDMA_CH0`.
struct Lcd {
    dpi: DsiDpi<'static>,
    /// Start addresses of the two `LCD_FB_BYTES` frame buffers.
    framebuffers: [usize; 2],
    hue: u32,
    vsync_timeouts: u32,
    baseline_rate_per_s: u64,
    capture_stats: LcdCaptureStats,
    _reset: Output<'static>,
    _backlight: Output<'static>,
}

/// DSI progress during CSI captures, from `DsiDpi::dma_interrupt_count`.
#[derive(Clone, Copy, Default)]
struct LcdCaptureStats {
    irqs: u32,
    min_irqs_per_capture: Option<u32>,
    min_rate_per_s: Option<u64>,
    captures_without_progress: u32,
    captures_with_underrun: u32,
}

impl Lcd {
    fn start(
        dsi: esp_hal::peripherals::MIPI_DSI<'static>,
        vdma_channel: esp_hal::peripherals::VDMA_CH0<'static>,
        reset: esp_hal::peripherals::GPIO27<'static>,
        backlight: esp_hal::peripherals::GPIO26<'static>,
        delay: &Delay,
    ) -> Self {
        let mut reset = Output::new(reset, Level::Low, OutputConfig::default());
        delay.delay_millis(10);
        reset.set_high();
        delay.delay_millis(120);
        let backlight = Output::new(backlight, Level::High, OutputConfig::default());

        let mut bus = MipiDsi::new(
            dsi,
            vdma_channel,
            DsiConfig::default()
                .with_num_data_lanes(DsiDataLanes::_2)
                .with_lane_bit_rate_mbps(1000.0)
                .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                    MipiDsiPhyPllRefclkSclk::Xtal,
                    0,
                ))
                .with_force_clock_lane_hs(false),
        )
        .expect("MipiDsi init failed");
        {
            let mut dbi = bus.dbi(0);
            for &(cmd, params) in EK79007_INIT {
                dbi.write_cmd(cmd, params).unwrap();
            }
            dbi.write_cmd(0x11, &[]).unwrap();
            delay.delay_millis(120);
            dbi.write_cmd(0x29, &[]).unwrap();
            delay.delay_millis(20);
        }

        let layout = Layout::from_size_align(LCD_FB_BYTES, PSRAM_DMA_ALIGNMENT).unwrap();
        let allocate = || {
            let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
            assert!(!ptr.is_null(), "LCD frame buffer allocation failed");
            unsafe { core::slice::from_raw_parts_mut(ptr, LCD_FB_BYTES) }
        };
        let fb1: &'static mut [u8] = allocate();
        let fb2: &'static mut [u8] = allocate();
        let framebuffers = [fb1.as_ptr() as usize, fb2.as_ptr() as usize];
        let fbs: [&'static mut [u8]; 2] = [fb1, fb2];

        let dpi = bus
            .dpi(
                DpiConfig {
                    virtual_channel: 0,
                    pixel_clock_mhz: 48.0,
                    dpi_clk_src: DpiClockSource::PllF240m,
                    in_color_format: ColorFormat::Rgb565,
                    out_color_format: ColorFormat::Rgb565,
                    timing: FrameTiming {
                        h_active: LCD_H_ACTIVE,
                        hsw: 10,
                        hbp: 120,
                        hfp: 120,
                        v_active: LCD_V_ACTIVE,
                        vsw: 1,
                        vbp: 20,
                        vfp: 20,
                    },
                },
                &fbs,
            )
            .expect("DPI init failed");

        Self {
            dpi,
            framebuffers,
            hue: 0,
            vsync_timeouts: 0,
            baseline_rate_per_s: 0,
            capture_stats: LcdCaptureStats::default(),
            _reset: reset,
            _backlight: backlight,
        }
    }

    /// One colour-wheel step of `lcd-mipi-dsi`, with a bounded vsync wait.
    fn render_step(&mut self) {
        let bridge = MIPI_DSI_BRIDGE::regs();
        let start = esp_hal::time::Instant::now();
        while !bridge.int_raw().read().vsync().bit_is_set() {
            if start.elapsed() > Duration::from_millis(LCD_VSYNC_TIMEOUT_MS) {
                self.vsync_timeouts += 1;
                break;
            }
        }
        bridge.int_clr().write(|w| w.vsync().clear_bit_by_one());

        let px = hue_to_rgb565(self.hue);
        let w = px as u32 | (px as u32) << 16;
        let c = w as u128 | (w as u128) << 32 | (w as u128) << 64 | (w as u128) << 96;
        for chunk in self.dpi.framebuffer_mut().as_chunks_mut::<16>().0 {
            *chunk = c.to_le_bytes();
        }
        self.dpi.commit();
        self.hue = (self.hue + 1) % 360;
    }

    /// Renders the next step and starts measuring DSI progress for a capture.
    fn before_capture(&mut self) -> (u32, esp_hal::time::Instant) {
        self.render_step();
        self.clear_underrun();
        (
            self.dpi.dma_interrupt_count(),
            esp_hal::time::Instant::now(),
        )
    }

    /// Returns the DSI DMA interrupts since [`Self::before_capture`], their
    /// rate per second, and whether the bridge FIFO ran empty.
    fn after_capture(
        &mut self,
        (irqs_before, start): (u32, esp_hal::time::Instant),
    ) -> (u32, u64, bool) {
        let irqs = self.dpi.dma_interrupt_count().wrapping_sub(irqs_before);
        let rate_per_s = irqs as u64 * 1_000_000 / start.elapsed().as_micros().max(1);
        let underrun = self.underrun_seen();
        let stats = &mut self.capture_stats;
        stats.irqs += irqs;
        stats.min_irqs_per_capture = Some(stats.min_irqs_per_capture.map_or(irqs, |m| m.min(irqs)));
        stats.min_rate_per_s = Some(
            stats
                .min_rate_per_s
                .map_or(rate_per_s, |m| m.min(rate_per_s)),
        );
        stats.captures_without_progress += (irqs == 0) as u32;
        stats.captures_with_underrun += underrun as u32;
        (irqs, rate_per_s, underrun)
    }

    fn clear_underrun(&self) {
        MIPI_DSI_BRIDGE::regs()
            .int_clr()
            .write(|w| w.underrun().clear_bit_by_one());
    }

    fn underrun_seen(&self) -> bool {
        MIPI_DSI_BRIDGE::regs()
            .int_raw()
            .read()
            .underrun()
            .bit_is_set()
    }

    fn overlaps(&self, buffer: &[u8]) -> bool {
        let start = buffer.as_ptr() as usize;
        let end = start + buffer.len();
        self.framebuffers
            .iter()
            .any(|&fb_start| start < fb_start + LCD_FB_BYTES && fb_start < end)
    }

    /// Runs the colour wheel alone, as the reference for the capture phase.
    fn baseline(&mut self) -> bool {
        self.clear_underrun();
        let timeouts_before = self.vsync_timeouts;
        let irqs_before = self.dpi.dma_interrupt_count();
        let start = esp_hal::time::Instant::now();
        for _ in 0..LCD_BASELINE_STEPS {
            self.render_step();
        }
        let elapsed_us = start.elapsed().as_micros().max(1);
        let irqs = self.dpi.dma_interrupt_count().wrapping_sub(irqs_before);
        let timeouts = self.vsync_timeouts - timeouts_before;
        let underrun = self.underrun_seen();
        self.baseline_rate_per_s = irqs as u64 * 1_000_000 / elapsed_us;
        println!(
            "LCD-only baseline: steps={} elapsed_ms={} steps_per_s_x10={} dsi_dma_irqs={} dsi_dma_irqs_per_s={} vsync_timeouts={} underrun={}",
            LCD_BASELINE_STEPS,
            elapsed_us / 1000,
            LCD_BASELINE_STEPS as u64 * 10_000_000 / elapsed_us,
            irqs,
            self.baseline_rate_per_s,
            timeouts,
            underrun
        );
        irqs > 0 && timeouts == 0
    }
}

fn hue_to_rgb565(hue: u32) -> u16 {
    let region = hue / 60;
    let frac = hue % 60;
    let (r, g, b) = match region {
        0 => (255, (frac * 255 / 60) as u8, 0),
        1 => ((255 - frac * 255 / 60) as u8, 255, 0),
        2 => (0, 255, (frac * 255 / 60) as u8),
        3 => (0, (255 - frac * 255 / 60) as u8, 255),
        4 => ((frac * 255 / 60) as u8, 0, 255),
        _ => (255, 0, (255 - frac * 255 / 60) as u8),
    };
    ((r as u16) >> 3) << 11 | ((g as u16) >> 2) << 5 | (b as u16) >> 3
}

/// EK79007 vendor init sequence, as in `lcd-mipi-dsi`.
static EK79007_INIT: &[(u8, &[u8])] = &[
    (0xB2, &[0x10]),
    (0x80, &[0x8B]),
    (0x81, &[0x78]),
    (0x82, &[0x84]),
    (0x83, &[0x88]),
    (0x84, &[0xA8]),
    (0x85, &[0xE3]),
    (0x86, &[0x88]),
];

/// Interrupt-matrix entries of `Interrupt::DMA` on both cores, 0 when unmapped.
fn dma_interrupt_map() -> (u32, u32) {
    let index = esp_hal::peripherals::Interrupt::DMA as usize;
    (
        INTERRUPT_CORE0::regs().core_0_intr_map(index).read().bits(),
        INTERRUPT_CORE1::regs().core_1_intr_map(index).read().bits(),
    )
}

/// Enable and signal-enable state of the two VDMA channels in use.
fn print_vdma_channel_state(label: &str) {
    let vdma = VDMA::regs();
    let (core0, core1) = dma_interrupt_map();
    println!(
        "{label}: chen0={:#x} ch0_signal_enable={:#x} ch1_signal_enable={:#x} ch1_status_enable={:#x} ch1_routable={:#x} ch0_status={:#x} ch1_status={:#x} dma_irq_map core0={} core1={}",
        vdma.chen0().read().bits() & 0xf,
        vdma.ch(0).intsignal_enable0().read().bits(),
        vdma.ch(1).intsignal_enable0().read().bits(),
        vdma.ch(1).intstatus_enable0().read().bits(),
        routable_signal_enable(1),
        vdma.ch(0).intstatus0().read().bits(),
        vdma.ch(1).intstatus0().read().bits(),
        core0,
        core1
    );
}

/// Prints the DSI side of the run, then drops CSI and DSI in turn and checks
/// that each leaves the other's channel, and no stale interrupt state, behind.
fn coexistence_teardown(capture: CsiCapture<'_>, mut lcd: Lcd) -> bool {
    let stats = lcd.capture_stats;
    println!(
        "DSI DMA interrupts during CSI captures: total={} min_per_capture={:?} min_per_s={:?} lcd_only_per_s={} captures_without_progress={} captures_with_underrun={} vsync_timeouts={}",
        stats.irqs,
        stats.min_irqs_per_capture,
        stats.min_rate_per_s,
        lcd.baseline_rate_per_s,
        stats.captures_without_progress,
        stats.captures_with_underrun,
        lcd.vsync_timeouts
    );
    let mut ok = stats.captures_without_progress == 0 && lcd.vsync_timeouts == 0;

    print_vdma_channel_state("before-csi-drop");
    drop(capture);
    let vdma = VDMA::regs();
    let chen = vdma.chen0().read().bits();
    let csi_released = chen & 0b10 == 0
        && routable_signal_enable(1) == 0
        && vdma.ch(1).intstatus0().read().bits() == 0;
    let dsi_running = chen & 0b01 != 0;
    print_vdma_channel_state("after-csi-drop");
    println!(
        "after-csi-drop: csi_channel_released={csi_released} dsi_channel_enabled={dsi_running}"
    );
    ok &= csi_released && dsi_running;

    let irqs_before = lcd.dpi.dma_interrupt_count();
    let timeouts_before = lcd.vsync_timeouts;
    let start = esp_hal::time::Instant::now();
    let mut steps = 0;
    while start.elapsed() < Duration::from_millis(LCD_TAIL_MS) {
        lcd.render_step();
        steps += 1;
    }
    let tail_irqs = lcd.dpi.dma_interrupt_count().wrapping_sub(irqs_before);
    let tail_timeouts = lcd.vsync_timeouts - timeouts_before;
    println!(
        "LCD after CSI drop: {}ms steps={} dsi_dma_irqs={} vsync_timeouts={}",
        LCD_TAIL_MS, steps, tail_irqs, tail_timeouts
    );
    ok &= tail_irqs > 0 && tail_timeouts == 0;

    drop(lcd);
    // The last VDMA guard is gone, so the controller registers are no longer
    // readable; with the clock gated no channel can raise the interrupt.
    let vdma_clock = HP_SYS_CLKRST::regs()
        .soc_clk_ctrl1()
        .read()
        .gdma_sys_clk_en()
        .bit();
    println!("after-dsi-drop: vdma_clock_enabled={vdma_clock}");
    ok && !vdma_clock
}

/// Read back from `INTSIGNAL_ENABLE0` and `INTSTATUS_ENABLE0` after writing 0:
/// the bits the PAC leaves undefined plus bit 25. None of them latched a status
/// in any capture.
const VDMA_ENABLE_STUCK_AT_ONE: u32 = 0x07c0_8004;

/// Signal-enable bits of `channel` that a driver set and that can raise the
/// interrupt.
fn routable_signal_enable(channel: usize) -> u32 {
    let ch = VDMA::regs().ch(channel);
    ch.intsignal_enable0().read().bits()
        & ch.intstatus_enable0().read().bits()
        & !VDMA_ENABLE_STUCK_AT_ONE
}

/// Same names and changes as `VDMA_MEM2MEM_DMA_PROFILE` in `vdma-mem2mem-psram`.
fn selected_dma_profile() -> (&'static str, DmaAxiConfig) {
    let idf = DmaAxiConfig::ESP_IDF;
    match option_env!("CSI_DMA_PROFILE") {
        None | Some("dst-osr-1") => (
            "dst-osr-1",
            DmaAxiConfig {
                dst_outstanding_requests: 1,
                ..idf
            },
        ),
        Some("dst-awlen-4") => (
            "dst-awlen-4",
            DmaAxiConfig {
                dst_burst_len: 4,
                ..idf
            },
        ),
        Some("src-osr-1") => (
            "src-osr-1",
            DmaAxiConfig {
                src_outstanding_requests: 1,
                ..idf
            },
        ),
        Some("idf") => ("idf", idf),
        Some(other) => panic!(
            "CSI_DMA_PROFILE must be dst-osr-1, dst-awlen-4, src-osr-1, or idf, got '{other}'"
        ),
    }
}

/// Raw CSI host, CSI bridge and VDMA register values, read through the PAC.
#[derive(Clone, Copy)]
struct CaptureDebugSnapshot {
    dma_channel_id: u8,
    host_phy_stopstate_raw: u32,
    host_phy_rx_raw: u32,
    host_int_st_main_raw: u32,
    bridge_csi_en_raw: u32,
    bridge_frame_cfg_raw: u32,
    bridge_data_type_cfg_raw: u32,
    bridge_host_cm_ctrl_raw: u32,
    bridge_buf_flow_ctl_raw: u32,
    bridge_dma_req_cfg_raw: u32,
    bridge_dmablk_size_raw: u32,
    bridge_int_raw_raw: u32,
    bridge_int_st_raw: u32,
    bridge_int_ena_raw: u32,
    dma_chen0_raw: u32,
    dma_chen1_raw: u32,
    dma_controller_cfg0_raw: u32,
    dma_controller_reset0_raw: u32,
    dma_controller_lowpower_cfg0_raw: u32,
    dma_controller_lowpower_cfg1_raw: u32,
    dma_cfg0_raw: u32,
    dma_cfg1_raw: u32,
    dma_ctl0_raw: u32,
    dma_ctl1_raw: u32,
    dma_sar0_raw: u32,
    dma_dar0_raw: u32,
    dma_block_ts0_raw: u32,
    dma_sstatar0_raw: u32,
    dma_dstatar0_raw: u32,
    dma_sstat0_raw: u32,
    dma_dstat0_raw: u32,
    dma_status0_raw: u32,
    dma_status1_raw: u32,
    dma_intstatus_enable0_raw: u32,
    dma_intstatus0_raw: u32,
}

/// The capture runs on `VDMA_CH1`.
const CSI_DMA_CHANNEL: usize = 1;

/// Only valid while a `CsiCapture` lives, which keeps the CSI and VDMA clocks
/// on.
fn debug_snapshot() -> CaptureDebugSnapshot {
    let host = MIPI_CSI_HOST::regs();
    let bridge = MIPI_CSI_BRIDGE::regs();
    let vdma = VDMA::regs();
    let ch = vdma.ch(CSI_DMA_CHANNEL);
    CaptureDebugSnapshot {
        dma_channel_id: CSI_DMA_CHANNEL as u8,
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
        dma_chen0_raw: vdma.chen0().read().bits(),
        dma_chen1_raw: vdma.chen1().read().bits(),
        dma_controller_cfg0_raw: vdma.cfg0().read().bits(),
        dma_controller_reset0_raw: vdma.reset0().read().bits(),
        dma_controller_lowpower_cfg0_raw: vdma.lowpower_cfg0().read().bits(),
        dma_controller_lowpower_cfg1_raw: vdma.lowpower_cfg1().read().bits(),
        dma_cfg0_raw: ch.cfg0().read().bits(),
        dma_cfg1_raw: ch.cfg1().read().bits(),
        dma_ctl0_raw: ch.ctl0().read().bits(),
        dma_ctl1_raw: ch.ctl1().read().bits(),
        dma_sar0_raw: ch.sar0().read().bits(),
        dma_dar0_raw: ch.dar0().read().bits(),
        dma_block_ts0_raw: ch.block_ts0().read().bits(),
        dma_sstatar0_raw: ch.sstatar0().read().bits(),
        dma_dstatar0_raw: ch.dstatar0().read().bits(),
        dma_sstat0_raw: ch.sstat0().read().bits(),
        dma_dstat0_raw: ch.dstat0().read().bits(),
        dma_status0_raw: ch.status0().read().bits(),
        dma_status1_raw: ch.status1().read().bits(),
        dma_intstatus_enable0_raw: ch.intstatus_enable0().read().bits(),
        dma_intstatus0_raw: ch.intstatus0().read().bits(),
    }
}

/// Decodes the channel registers that the AXI profile and the CSI flow setup
/// program. `CTL0` and `CTL1` are only written when a capture is armed.
fn print_dma_axi_readback(stage: &str, snapshot: &CaptureDebugSnapshot) {
    let cfg1 = snapshot.dma_cfg1_raw;
    let ctl0 = snapshot.dma_ctl0_raw;
    let ctl1 = snapshot.dma_ctl1_raw;
    println!(
        "dma-axi({stage}): tt_fc={} hs_sel_src={} src_per={} src_outstanding={} dst_outstanding={} sms={} dms={} sinc_fixed={} dinc_fixed={} src_width={} dst_width={} src_msize={} dst_msize={} arlen_en={} arlen={} awlen_en={} awlen={}",
        cfg1 & 0x7,
        (cfg1 >> 3) & 1,
        (cfg1 >> 7) & 0x3,
        ((cfg1 >> 23) & 0xf) + 1,
        ((cfg1 >> 27) & 0xf) + 1,
        ctl0 & 1,
        (ctl0 >> 2) & 1,
        (ctl0 >> 4) & 1,
        (ctl0 >> 6) & 1,
        (ctl0 >> 8) & 0x7,
        (ctl0 >> 11) & 0x7,
        (ctl0 >> 14) & 0xf,
        (ctl0 >> 18) & 0xf,
        (ctl1 >> 6) & 1,
        (ctl1 >> 7) & 0xff,
        (ctl1 >> 15) & 1,
        (ctl1 >> 16) & 0xff,
    );
}

// Words after the capture slice. The driver panics when the DMA destination
// address passes the slice, so a changed guard word means that `DAR`
// under-reported the writes.
const GUARD_BYTES: usize = 4096;
const PSRAM_DMA_ALIGNMENT: usize = 128;
const SENTINEL_SEED: u32 = 0xA5C3_5A3C;

fn enable_mipi_dphy_power(delay: &Delay) {
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(MIPI_DPHY_LDO_CTRL) });
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits(MIPI_DPHY_LDO_ANA_2P5V) });

    // Let the CSI PHY rail settle before sensor checks and CSI init.
    delay.delay_micros(500);
}

/// Cycles of the combined cases after the first pass of `CSI_DPHY_LIFECYCLE`.
const DPHY_LIFECYCLE_REPEATS: u32 = 3;

/// The PMU drives `VDD_MIPI_DPHY` (VO3 `xpd`, bit 8 of `ext_ldo_p0_0p2a`).
fn dphy_rail_on() -> bool {
    PMU::regs().ext_ldo_p0_0p2a().read().bits() & (1 << 8) != 0
}

fn lifecycle_rail(step: &str, expect_on: bool) -> bool {
    let on = dphy_rail_on();
    let ok = on == expect_on;
    println!(
        "dphy-lifecycle[{step}]: rail_on={on} expected={expect_on} {}",
        if ok { "ok" } else { "FAIL" }
    );
    ok
}

fn lifecycle_csi<'d>(csi: MIPI_CSI<'d>, channel: VDMA_CH1<'d>) -> CsiCapture<'d> {
    let capture = MipiCsi::new(
        csi,
        Config::default()
            .with_num_data_lanes(SC2336_LANES)
            .with_lane_bit_rate_mbps(SC2336_LANE_BIT_RATE_MBPS),
    )
    .unwrap()
    .with_dma(
        channel,
        CaptureConfig::default()
            .with_frame_width(SC2336_FRAME_WIDTH)
            .with_frame_height(SC2336_FRAME_HEIGHT)
            .with_pixel_format(CapturePixelFormat::Raw8)
            .with_timeout(Duration::from_millis(500)),
    )
    .unwrap();
    apply_idf_default_bridge_data_type_filter();
    configure_isp_like_idf(false);
    capture
}

fn lifecycle_frame(
    step: &str,
    capture: &mut CsiCapture<'_>,
    i2c: &mut I2c<'_, Blocking>,
    buffer: &mut [u8],
) -> bool {
    let result = capture.receive_with(&mut buffer[..SC2336_FRAME_BYTES], || {
        let _ = set_sc2336_stream(i2c, true);
    });
    let _ = set_sc2336_stream(i2c, false);
    let ok = matches!(result, Ok(frame) if frame.received_bytes == SC2336_FRAME_BYTES);
    println!(
        "dphy-lifecycle[{step}]: csi_frame={:?} {}",
        result.map(|frame| frame.received_bytes),
        if ok { "ok" } else { "FAIL" }
    );
    ok
}

fn lifecycle_dsi<'d>(dsi: MIPI_DSI<'d>, channel: VDMA_CH0<'d>) -> MipiDsi<'d> {
    MipiDsi::new(
        dsi,
        channel,
        DsiConfig::default()
            .with_num_data_lanes(DsiDataLanes::_2)
            .with_lane_bit_rate_mbps(1000.0)
            .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                MipiDsiPhyPllRefclkSclk::Xtal,
                0,
            )),
    )
    .expect("MipiDsi init failed")
}

/// Only read while a `MipiDsi` lives, so that the DSI host is clocked.
fn lifecycle_dsi_locked(step: &str) -> bool {
    let ok = MIPI_DSI_HOST::regs()
        .phy_status()
        .read()
        .phy_lock()
        .bit_is_set();
    println!(
        "dphy-lifecycle[{step}]: dsi_pll_locked={ok} {}",
        if ok { "ok" } else { "FAIL" }
    );
    ok
}

/// Creates CSI and DSI in the order `csi_first`, and drops them in the order
/// `drop_csi_first`, checking the rail and both drivers after every step.
fn lifecycle_pair(
    label: &str,
    csi: &mut MIPI_CSI<'_>,
    csi_channel: &mut VDMA_CH1<'_>,
    dsi: &mut MIPI_DSI<'_>,
    dsi_channel: &mut VDMA_CH0<'_>,
    i2c: &mut I2c<'_, Blocking>,
    buffer: &mut [u8],
    csi_first: bool,
    drop_csi_first: bool,
) -> bool {
    let mut ok = true;
    let (mut capture, bus) = if csi_first {
        let capture = lifecycle_csi(csi.reborrow(), csi_channel.reborrow());
        ok &= lifecycle_rail(&format!("{label}/csi-up"), true);
        (
            capture,
            lifecycle_dsi(dsi.reborrow(), dsi_channel.reborrow()),
        )
    } else {
        let bus = lifecycle_dsi(dsi.reborrow(), dsi_channel.reborrow());
        ok &= lifecycle_rail(&format!("{label}/dsi-up"), true);
        (lifecycle_csi(csi.reborrow(), csi_channel.reborrow()), bus)
    };
    ok &= lifecycle_rail(&format!("{label}/both-up"), true);
    ok &= lifecycle_frame(&format!("{label}/both-up"), &mut capture, i2c, buffer);
    ok &= lifecycle_dsi_locked(&format!("{label}/both-up"));
    if drop_csi_first {
        drop(capture);
        ok &= lifecycle_rail(&format!("{label}/csi-dropped"), true);
        ok &= lifecycle_dsi_locked(&format!("{label}/csi-dropped"));
        drop(bus);
    } else {
        drop(bus);
        ok &= lifecycle_rail(&format!("{label}/dsi-dropped"), true);
        ok &= lifecycle_frame(&format!("{label}/dsi-dropped"), &mut capture, i2c, buffer);
        drop(capture);
    }
    ok &= lifecycle_rail(&format!("{label}/both-dropped"), false);
    ok
}

fn run_dphy_lifecycle(
    mut csi: MIPI_CSI<'_>,
    mut csi_channel: VDMA_CH1<'_>,
    mut dsi: MIPI_DSI<'_>,
    mut dsi_channel: VDMA_CH0<'_>,
    i2c: &mut I2c<'_, Blocking>,
) -> bool {
    let buffer = allocate_capture_buffer(CaptureMode::Raw8Psram);
    let mut ok = lifecycle_rail("start", false);

    let mut capture = lifecycle_csi(csi.reborrow(), csi_channel.reborrow());
    ok &= lifecycle_rail("1-csi-alone/up", true);
    ok &= lifecycle_frame("1-csi-alone/up", &mut capture, i2c, buffer);
    drop(capture);
    ok &= lifecycle_rail("1-csi-alone/dropped", false);

    let bus = lifecycle_dsi(dsi.reborrow(), dsi_channel.reborrow());
    ok &= lifecycle_rail("2-dsi-alone/up", true);
    ok &= lifecycle_dsi_locked("2-dsi-alone/up");
    drop(bus);
    ok &= lifecycle_rail("2-dsi-alone/dropped", false);

    let cases = [
        ("3-csi-dsi-drop-dsi", true, false),
        ("4-csi-dsi-drop-csi", true, true),
        ("5-dsi-csi-drop-csi", false, true),
        ("6-dsi-csi-drop-dsi", false, false),
    ];
    for cycle in 0..=DPHY_LIFECYCLE_REPEATS {
        for (name, csi_first, drop_csi_first) in cases {
            let label = if cycle == 0 {
                String::from(name)
            } else {
                format!("7-repeat{cycle}-{name}")
            };
            ok &= lifecycle_pair(
                &label,
                &mut csi,
                &mut csi_channel,
                &mut dsi,
                &mut dsi_channel,
                i2c,
                buffer,
                csi_first,
                drop_csi_first,
            );
        }
    }
    ok
}

fn read_reg_u8(i2c: &mut I2c<'_, Blocking>, reg: u16) -> Result<u8, i2c::master::Error> {
    let mut value = [0u8; 1];
    i2c.write_read(SC2336_SCCB_ADDR, &reg.to_be_bytes(), &mut value)?;
    Ok(value[0])
}

fn write_reg_u8(
    i2c: &mut I2c<'_, Blocking>,
    reg: u16,
    value: u8,
) -> Result<(), i2c::master::Error> {
    let reg = reg.to_be_bytes();
    let tx = [reg[0], reg[1], value];
    i2c.write(SC2336_SCCB_ADDR, &tx)
}

fn verify_sc2336_id(i2c: &mut I2c<'_, Blocking>) -> Result<u16, i2c::master::Error> {
    let pid_h = read_reg_u8(i2c, SC2336_REG_SENSOR_ID_H)?;
    let pid_l = read_reg_u8(i2c, SC2336_REG_SENSOR_ID_L)?;
    let pid = ((pid_h as u16) << 8) | pid_l as u16;

    println!(
        "SC2336 ID regs: [0x{:04x}]=0x{:02x}, [0x{:04x}]=0x{:02x}",
        SC2336_REG_SENSOR_ID_H, pid_h, SC2336_REG_SENSOR_ID_L, pid_l
    );
    println!(
        "SC2336 PID observed=0x{:04x}, expected=0x{:04x}",
        pid, SC2336_EXPECTED_PID
    );
    Ok(pid)
}

fn program_sc2336_mode_1024x600_raw8(
    i2c: &mut I2c<'_, Blocking>,
    delay: &Delay,
) -> Result<(), i2c::master::Error> {
    for &(reg, val) in SC2336_MODE_1024X600_RAW8_30FPS {
        write_reg_u8(i2c, reg, val)?;
    }
    delay.delay_millis(5);
    Ok(())
}

fn set_sc2336_stream(i2c: &mut I2c<'_, Blocking>, enable: bool) -> Result<(), i2c::master::Error> {
    write_reg_u8(
        i2c,
        SC2336_REG_STREAM_CTRL,
        if enable { 0x01 } else { 0x00 },
    )
}

fn print_sc2336_regs(
    i2c: &mut I2c<'_, Blocking>,
    label: &str,
    regs: &[u16],
) -> Result<(), i2c::master::Error> {
    println!("SC2336 register readback ({label})");
    for &reg in regs {
        let value = read_reg_u8(i2c, reg)?;
        println!("  [0x{:04x}] = 0x{:02x}", reg, value);
    }
    Ok(())
}

fn checksum32(data: &[u8]) -> u32 {
    data.iter()
        .fold(0u32, |acc, &byte| acc.wrapping_mul(16777619) ^ byte as u32)
}

fn print_sample_bytes(data: &[u8], sample_len: usize) {
    let n = core::cmp::min(sample_len, data.len());
    print!("sample[0..{}] =", n);
    for b in &data[..n] {
        print!(" {:02x}", b);
    }
    println!();
}

fn print_capture_status(prefix: &str, status: &esp_hal::mipi_csi::CaptureStatus) {
    println!(
        "{} host: raw=0x{:08x} phy_fatal={} pkt_fatal={} bndry_fatal={} seq_fatal={} crc_fatal={} pld_crc_fatal={} data_id={} ecc_corrected={} phy_err={}",
        prefix,
        status.host_errors.raw_main_status,
        status.host_errors.phy_fatal,
        status.host_errors.packet_fatal,
        status.host_errors.boundary_frame_fatal,
        status.host_errors.sequence_frame_fatal,
        status.host_errors.crc_frame_fatal,
        status.host_errors.payload_crc_fatal,
        status.host_errors.data_id_error,
        status.host_errors.ecc_corrected,
        status.host_errors.phy_error
    );
    println!(
        "{} bridge: raw=0x{:08x} vadr_gt={} vadr_lt={} discard={} buf_overrun={} fifo_ovf={} dma_cfg_updated={}",
        prefix,
        status.bridge_events.raw_status,
        status.bridge_events.vadr_num_gt,
        status.bridge_events.vadr_num_lt,
        status.bridge_events.discard,
        status.bridge_events.csi_buf_overrun,
        status.bridge_events.csi_async_fifo_ovf,
        status.bridge_events.dma_cfg_has_updated
    );
    println!(
        "{} dma: raw=0x{:08x} channel_enabled={} transfer_done={} any_error={} channel_stopped={} completed_units={} fifo_remaining_units={}",
        prefix,
        status.dma_status.raw_status,
        status.dma_status.channel_enabled,
        status.dma_status.transfer_done(),
        status.dma_status.any_error(),
        status.dma_status.channel_stopped(),
        status.dma_status.completed_transfer_units,
        status.dma_status.fifo_remaining_units
    );
}

fn print_capture_debug_snapshot(stage: &str, snapshot: &CaptureDebugSnapshot) {
    println!("--- CSI DEBUG SNAPSHOT: {stage} ---");
    println!(
        "host: phy_stopstate=0x{:08x} phy_rx=0x{:08x} int_st_main=0x{:08x}",
        snapshot.host_phy_stopstate_raw, snapshot.host_phy_rx_raw, snapshot.host_int_st_main_raw
    );
    println!(
        "bridge: csi_en=0x{:08x} frame_cfg=0x{:08x} data_type_cfg=0x{:08x} host_cm_ctrl=0x{:08x} buf_flow_ctl=0x{:08x}",
        snapshot.bridge_csi_en_raw,
        snapshot.bridge_frame_cfg_raw,
        snapshot.bridge_data_type_cfg_raw,
        snapshot.bridge_host_cm_ctrl_raw,
        snapshot.bridge_buf_flow_ctl_raw
    );
    let frame_vadr_num = snapshot.bridge_frame_cfg_raw & 0x0fff;
    let frame_hadr_num = (snapshot.bridge_frame_cfg_raw >> 12) & 0x0fff;
    let frame_has_hsync = ((snapshot.bridge_frame_cfg_raw >> 24) & 1) != 0;
    let frame_vadr_check = ((snapshot.bridge_frame_cfg_raw >> 25) & 1) != 0;
    let dt_min = snapshot.bridge_data_type_cfg_raw & 0x3f;
    let dt_max = (snapshot.bridge_data_type_cfg_raw >> 8) & 0x3f;
    println!(
        "bridge-decode: hadr_num={} vadr_num={} has_hsync_e={} vadr_num_check={} dt_min=0x{:02x} dt_max=0x{:02x}",
        frame_hadr_num, frame_vadr_num, frame_has_hsync, frame_vadr_check, dt_min, dt_max
    );
    let cm_en = (snapshot.bridge_host_cm_ctrl_raw & 0x1) != 0;
    let cm_bypass = ((snapshot.bridge_host_cm_ctrl_raw >> 1) & 0x1) != 0;
    let cm_rx = (snapshot.bridge_host_cm_ctrl_raw >> 2) & 0x3;
    let cm_tx = (snapshot.bridge_host_cm_ctrl_raw >> 9) & 0x3;
    println!(
        "bridge-cm: cm_en={} cm_bypass={} cm_rx={} cm_tx={}",
        cm_en, cm_bypass, cm_rx, cm_tx
    );
    println!(
        "bridge: dma_req_cfg=0x{:08x} dmablk_size=0x{:08x} int_raw=0x{:08x} int_st=0x{:08x} int_ena=0x{:08x}",
        snapshot.bridge_dma_req_cfg_raw,
        snapshot.bridge_dmablk_size_raw,
        snapshot.bridge_int_raw_raw,
        snapshot.bridge_int_st_raw,
        snapshot.bridge_int_ena_raw
    );
    let dma_burst_len = snapshot.bridge_dma_req_cfg_raw & 0x0fff;
    let dma_cfg_upd_by_blk = ((snapshot.bridge_dma_req_cfg_raw >> 12) & 1) != 0;
    let dma_force_rd_status = ((snapshot.bridge_dma_req_cfg_raw >> 16) & 1) != 0;
    let csi_bridge_flow_ctrl = ((snapshot.bridge_dma_req_cfg_raw >> 17) & 1) != 0;
    let dmablk_size = snapshot.bridge_dmablk_size_raw & 0x1fff;
    let words_per_bridge_block = (dmablk_size as u64).saturating_mul(dma_burst_len as u64);
    println!(
        "bridge-decode: dma_burst_len={} dma_cfg_upd_by_blk={} dma_force_rd_status={} csi_bridge_flow_ctrl={} dmablk_size={} (~{} x64-bit words/block)",
        dma_burst_len,
        dma_cfg_upd_by_blk,
        dma_force_rd_status,
        csi_bridge_flow_ctrl,
        dmablk_size,
        words_per_bridge_block
    );
    println!(
        "dma-global: ch={} cfg0=0x{:08x} reset0=0x{:08x} lowpower_cfg0=0x{:08x} lowpower_cfg1=0x{:08x} chen0=0x{:08x} chen1=0x{:08x}",
        snapshot.dma_channel_id,
        snapshot.dma_controller_cfg0_raw,
        snapshot.dma_controller_reset0_raw,
        snapshot.dma_controller_lowpower_cfg0_raw,
        snapshot.dma_controller_lowpower_cfg1_raw,
        snapshot.dma_chen0_raw,
        snapshot.dma_chen1_raw
    );
    let lowpower_gbl_cslp_en = (snapshot.dma_controller_lowpower_cfg0_raw & 0x1) != 0;
    let lowpower_chnl_cslp_en = ((snapshot.dma_controller_lowpower_cfg0_raw >> 1) & 0x1) != 0;
    let lowpower_sbiu_cslp_en = ((snapshot.dma_controller_lowpower_cfg0_raw >> 2) & 0x1) != 0;
    let lowpower_mxif_cslp_en = ((snapshot.dma_controller_lowpower_cfg0_raw >> 3) & 0x1) != 0;
    println!(
        "dma-lowpower: gbl_cslp_en={} chnl_cslp_en={} sbiu_cslp_en={} mxif_cslp_en={}",
        lowpower_gbl_cslp_en, lowpower_chnl_cslp_en, lowpower_sbiu_cslp_en, lowpower_mxif_cslp_en
    );
    println!(
        "dma-ch: cfg0=0x{:08x} cfg1=0x{:08x} ctl0=0x{:08x} ctl1=0x{:08x}",
        snapshot.dma_cfg0_raw, snapshot.dma_cfg1_raw, snapshot.dma_ctl0_raw, snapshot.dma_ctl1_raw
    );
    let dma_ctl1_src_stat_en = ((snapshot.dma_ctl1_raw >> 24) & 1) != 0;
    let dma_ctl1_dst_stat_en = ((snapshot.dma_ctl1_raw >> 25) & 1) != 0;
    let dma_ctl1_ioc_blktfr = ((snapshot.dma_ctl1_raw >> 26) & 1) != 0;
    let dma_ctl1_shadow_last = ((snapshot.dma_ctl1_raw >> 30) & 1) != 0;
    let dma_ctl1_shadow_valid = ((snapshot.dma_ctl1_raw >> 31) & 1) != 0;
    println!(
        "dma-ch-decode: src_stat_en={} dst_stat_en={} ioc_blktfr={} shadow_last={} shadow_valid={}",
        dma_ctl1_src_stat_en,
        dma_ctl1_dst_stat_en,
        dma_ctl1_ioc_blktfr,
        dma_ctl1_shadow_last,
        dma_ctl1_shadow_valid
    );
    println!(
        "dma-ch: sar0=0x{:08x} dar0=0x{:08x} block_ts0=0x{:08x}",
        snapshot.dma_sar0_raw, snapshot.dma_dar0_raw, snapshot.dma_block_ts0_raw
    );
    println!(
        "dma-ch: sstatar0=0x{:08x} dstatar0=0x{:08x} sstat0=0x{:08x} dstat0=0x{:08x}",
        snapshot.dma_sstatar0_raw,
        snapshot.dma_dstatar0_raw,
        snapshot.dma_sstat0_raw,
        snapshot.dma_dstat0_raw
    );
    println!(
        "dma-ch: status0=0x{:08x} status1=0x{:08x} intstatus_enable0=0x{:08x} intstatus0=0x{:08x}",
        snapshot.dma_status0_raw,
        snapshot.dma_status1_raw,
        snapshot.dma_intstatus_enable0_raw,
        snapshot.dma_intstatus0_raw
    );
}

fn apply_idf_default_bridge_data_type_filter() {
    let bridge = esp_hal::peripherals::MIPI_CSI_BRIDGE::regs();
    bridge.data_type_cfg().write(|w| unsafe {
        w.data_type_min().bits(CSI_IDF_DEFAULT_DATA_TYPE_MIN);
        w.data_type_max().bits(CSI_IDF_DEFAULT_DATA_TYPE_MAX)
    });
}

#[derive(Clone, Copy)]
struct IspReadback {
    isp_clk_en_raw: u32,
    isp_cntl_raw: u32,
    isp_frame_cfg_raw: u32,
    isp_int_raw: u32,
    peri_clk_ctrl25_raw: u32,
    peri_clk_ctrl26_raw: u32,
    hp_rst_en0_raw: u32,
}

#[derive(Clone, Copy)]
struct IspReadSkipped {
    peri_clk_ctrl25_raw: u32,
    peri_clk_ctrl26_raw: u32,
    hp_rst_en0_raw: u32,
}

#[inline(always)]
fn read_mmio_u32(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
fn write_mmio_u32(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
}

fn read_isp_registers_with_safe_preconditions() -> Result<IspReadback, IspReadSkipped> {
    let clkrst = esp_hal::peripherals::HP_SYS_CLKRST::regs();

    let original_peri_clk_ctrl25 = clkrst.peri_clk_ctrl25().read().bits();
    let enabled_peri_clk_ctrl25 =
        original_peri_clk_ctrl25 | HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_EN_BIT;
    if enabled_peri_clk_ctrl25 != original_peri_clk_ctrl25 {
        clkrst
            .peri_clk_ctrl25()
            .modify(|_, w| unsafe { w.bits(enabled_peri_clk_ctrl25) });
    }

    let peri_clk_ctrl26_raw = clkrst.peri_clk_ctrl26().read().bits();
    let hp_rst_en0_raw = clkrst.hp_rst_en0().read().bits();
    if hp_rst_en0_raw & HP_SYS_CLKRST_HP_RST_EN0_RST_EN_ISP_BIT != 0 {
        if enabled_peri_clk_ctrl25 != original_peri_clk_ctrl25 {
            clkrst
                .peri_clk_ctrl25()
                .modify(|_, w| unsafe { w.bits(original_peri_clk_ctrl25) });
        }
        return Err(IspReadSkipped {
            peri_clk_ctrl25_raw: enabled_peri_clk_ctrl25,
            peri_clk_ctrl26_raw,
            hp_rst_en0_raw,
        });
    }

    let readback = IspReadback {
        isp_clk_en_raw: read_mmio_u32(ISP_CLK_EN_REG_ADDR),
        isp_cntl_raw: read_mmio_u32(ISP_CNTL_REG_ADDR),
        isp_frame_cfg_raw: read_mmio_u32(ISP_FRAME_CFG_REG_ADDR),
        isp_int_raw: read_mmio_u32(ISP_INT_RAW_REG_ADDR),
        peri_clk_ctrl25_raw: enabled_peri_clk_ctrl25,
        peri_clk_ctrl26_raw,
        hp_rst_en0_raw,
    };

    if enabled_peri_clk_ctrl25 != original_peri_clk_ctrl25 {
        clkrst
            .peri_clk_ctrl25()
            .modify(|_, w| unsafe { w.bits(original_peri_clk_ctrl25) });
    }

    Ok(readback)
}

fn print_isp_input_gate_readback(label: &str) -> Result<IspReadback, IspReadSkipped> {
    let readback = read_isp_registers_with_safe_preconditions();
    match readback {
        Ok(rb) => {
            let mipi_data_en = (rb.isp_cntl_raw & ISP_CNTL_MIPI_DATA_EN_BIT) != 0;
            let isp_en = (rb.isp_cntl_raw & ISP_CNTL_ISP_EN_BIT) != 0;
            let isp_in_src =
                (rb.isp_cntl_raw & ISP_CNTL_ISP_IN_SRC_MASK) >> ISP_CNTL_ISP_IN_SRC_SHIFT;
            let isp_data_type =
                (rb.isp_cntl_raw & ISP_CNTL_ISP_DATA_TYPE_MASK) >> ISP_CNTL_ISP_DATA_TYPE_SHIFT;
            let isp_out_type =
                (rb.isp_cntl_raw & ISP_CNTL_OUT_TYPE_MASK) >> ISP_CNTL_OUT_TYPE_SHIFT;
            let demosaic_en = (rb.isp_cntl_raw & ISP_CNTL_DEMOSAIC_EN_BIT) != 0;
            let rgb2yuv_en = (rb.isp_cntl_raw & ISP_CNTL_RGB2YUV_EN_BIT) != 0;
            let yuv2rgb_en = (rb.isp_cntl_raw & ISP_CNTL_YUV2RGB_EN_BIT) != 0;
            let vadr_num = (rb.isp_frame_cfg_raw & ISP_FRAME_CFG_VADR_NUM_MASK) + 1;
            let hadr_num = ((rb.isp_frame_cfg_raw & ISP_FRAME_CFG_HADR_NUM_MASK)
                >> ISP_FRAME_CFG_HADR_NUM_SHIFT)
                + 1;
            let hsync_start_exist =
                (rb.isp_frame_cfg_raw & ISP_FRAME_CFG_HSYNC_START_EXIST_BIT) != 0;
            let hsync_end_exist = (rb.isp_frame_cfg_raw & ISP_FRAME_CFG_HSYNC_END_EXIST_BIT) != 0;
            println!(
                "isp-path({}): clk_en=0x{:08x} cntl=0x{:08x} mipi_data_en={} isp_en={} isp_in_src={} isp_data_type={} isp_out_type={} demosaic_en={} rgb2yuv_en={} yuv2rgb_en={} frame_cfg=0x{:08x} hadr_num={} vadr_num={} hsync_start_exist={} hsync_end_exist={} peri_clk_ctrl25=0x{:08x} peri_clk_ctrl26=0x{:08x} hp_rst_en0=0x{:08x}",
                label,
                rb.isp_clk_en_raw,
                rb.isp_cntl_raw,
                mipi_data_en,
                isp_en,
                isp_in_src,
                isp_data_type,
                isp_out_type,
                demosaic_en,
                rgb2yuv_en,
                yuv2rgb_en,
                rb.isp_frame_cfg_raw,
                hadr_num,
                vadr_num,
                hsync_start_exist,
                hsync_end_exist,
                rb.peri_clk_ctrl25_raw,
                rb.peri_clk_ctrl26_raw,
                rb.hp_rst_en0_raw
            );
            print!(
                "isp-path({}): int_raw=0x{:08x} int_ena=0x{:08x}",
                label,
                rb.isp_int_raw,
                read_mmio_u32(ISP_INT_ENA_REG_ADDR)
            );
            print_bit_names(rb.isp_int_raw, ISP_INT_NAMES);
            println!(" errors=0x{:08x}", rb.isp_int_raw & ISP_INT_ERROR_MASK);
        }
        Err(skipped) => {
            println!(
                "isp-path({}): skipped ISP read (rst_en_isp asserted), peri_clk_ctrl25=0x{:08x}, peri_clk_ctrl26=0x{:08x}, hp_rst_en0=0x{:08x}",
                label,
                skipped.peri_clk_ctrl25_raw,
                skipped.peri_clk_ctrl26_raw,
                skipped.hp_rst_en0_raw
            );
        }
    }
    readback
}

/// Programs the ISP like `esp_isp_new_processor` and `esp_isp_enable` do for
/// the ESP-IDF `mipi_isp_dsi` example (RAW8 in, RGB565 out), or like
/// `esp_isp_new_processor` alone with `bypass_isp = true`.
fn configure_isp_like_idf(isp_enabled: bool) {
    let clkrst = esp_hal::peripherals::HP_SYS_CLKRST::regs();

    // Same clock and reset order as `s_isp_claim_processor` plus the 80 MHz
    // PLL240 divider used by `esp_isp_new_processor`.
    clkrst.peri_clk_ctrl25().modify(|r, w| unsafe {
        let mut bits = r.bits();
        bits |= HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_EN_BIT;
        bits = (bits & !HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_MASK)
            | HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_SRC_SEL_PLL240;
        w.bits(bits)
    });
    clkrst.peri_clk_ctrl26().modify(|r, w| unsafe {
        let mut bits = r.bits();
        bits = (bits & !HP_SYS_CLKRST_PERI_CLK_CTRL26_ISP_CLK_DIV_NUM_MASK)
            | HP_SYS_CLKRST_PERI_CLK_CTRL26_ISP_CLK_DIV_NUM_80MHZ_FROM_240MHZ;
        w.bits(bits)
    });

    clkrst
        .hp_rst_en0()
        .modify(|r, w| unsafe { w.bits(r.bits() | HP_SYS_CLKRST_HP_RST_EN0_RST_EN_ISP_BIT) });
    clkrst
        .hp_rst_en0()
        .modify(|r, w| unsafe { w.bits(r.bits() & !HP_SYS_CLKRST_HP_RST_EN0_RST_EN_ISP_BIT) });

    // `isp_ll_init`, built for chip revision v3.0 or later like the example.
    write_mmio_u32(ISP_CNTL_REG_ADDR, 0);
    write_mmio_u32(ISP_INT_CLR_REG_ADDR, u32::MAX);
    write_mmio_u32(ISP_INT_ENA_REG_ADDR, 0);

    // Bypass skips the color format fields.
    let mut cntl = 0;
    if isp_enabled {
        cntl = ISP_CNTL_DEMOSAIC_EN_BIT
            | ISP_CNTL_RGB2YUV_EN_BIT
            | ISP_CNTL_YUV2RGB_EN_BIT
            | ISP_CNTL_ISP_DATA_TYPE_RAW8
            | ISP_CNTL_OUT_TYPE_RGB565;
        write_mmio_u32(ISP_CNTL_REG_ADDR, cntl);
    }
    write_mmio_u32(ISP_CLK_EN_REG_ADDR, ISP_CLK_EN_BIT);
    write_mmio_u32(ISP_INT_ENA_REG_ADDR, ISP_INT_ERROR_MASK);

    // `mipi_data_en` gates the CSI input into the ISP, also in bypass.
    cntl |= ISP_CNTL_ISP_IN_SRC_CSI | ISP_CNTL_MIPI_DATA_EN_BIT;
    write_mmio_u32(ISP_CNTL_REG_ADDR, cntl);

    // In bypass the input width is counted in 32-bit words. The example has no
    // line start or line end packets.
    let hadr_num = if isp_enabled {
        SC2336_FRAME_WIDTH as u32
    } else {
        (SC2336_FRAME_WIDTH as u32 * 8).div_ceil(32)
    };
    let frame_cfg = ((SC2336_FRAME_HEIGHT as u32 - 1) & ISP_FRAME_CFG_VADR_NUM_MASK)
        | (((hadr_num - 1) << ISP_FRAME_CFG_HADR_NUM_SHIFT) & ISP_FRAME_CFG_HADR_NUM_MASK);
    write_mmio_u32(ISP_FRAME_CFG_REG_ADDR, frame_cfg);

    // The bayer order and the YUV standard keep their reset value 0, which is
    // what the example configuration writes.
    let shadow = read_mmio_u32(ISP_SHADOW_REG_CTRL_REG_ADDR);
    write_mmio_u32(
        ISP_SHADOW_REG_CTRL_REG_ADDR,
        (shadow & !ISP_SHADOW_UPDATE_SEL_MASK) | ISP_SHADOW_UPDATE_ONLY_NEXT_VSYNC,
    );

    // `esp_isp_enable` only sets `ISP_EN`, and refuses to run in bypass.
    if isp_enabled {
        write_mmio_u32(ISP_CNTL_REG_ADDR, cntl | ISP_CNTL_ISP_EN_BIT);
    }
    println!(
        "isp-config: isp_enabled={} cntl=0x{:08x} frame_cfg=0x{:08x} hadr_num={} vadr_num={}",
        isp_enabled,
        read_mmio_u32(ISP_CNTL_REG_ADDR),
        frame_cfg,
        hadr_num,
        SC2336_FRAME_HEIGHT
    );
}

fn print_bit_names(raw: u32, names: &[(u32, &str)]) {
    print!(" [");
    let mut known = 0;
    for &(bit, name) in names {
        known |= 1 << bit;
        if raw & (1 << bit) != 0 {
            print!(" {name}");
        }
    }
    if raw & !known != 0 {
        print!(" other=0x{:08x}", raw & !known);
    }
    print!(" ]");
}

fn clear_isp_interrupts() {
    write_mmio_u32(ISP_INT_CLR_REG_ADDR, u32::MAX);
}

/// Same masks as `mipi_csi_host_ll_enable_intr(.., MIPI_CSI_HOST_LL_INTR_ERR_ALL, true)`
/// in ESP-IDF, after reading the clear-on-read status registers.
fn unmask_csi_host_errors() {
    let host = MIPI_CSI_HOST::regs();
    let _ = (
        host.int_st_phy_fatal().read().bits(),
        host.int_st_pkt_fatal().read().bits(),
        host.int_st_bndry_frame_fatal().read().bits(),
        host.int_st_seq_frame_fatal().read().bits(),
        host.int_st_crc_frame_fatal().read().bits(),
        host.int_st_pld_crc_fatal().read().bits(),
        host.int_st_data_id().read().bits(),
        host.int_st_ecc_corrected().read().bits(),
        host.int_st_phy().read().bits(),
    );
    host.int_msk_phy_fatal().write(|w| unsafe { w.bits(0x3) });
    host.int_msk_pkt_fatal().write(|w| unsafe { w.bits(0x3) });
    host.int_msk_bndry_frame_fatal()
        .write(|w| unsafe { w.bits(0xffff) });
    host.int_msk_seq_frame_fatal()
        .write(|w| unsafe { w.bits(0xffff) });
    host.int_msk_crc_frame_fatal()
        .write(|w| unsafe { w.bits(0xffff) });
    host.int_msk_pld_crc_fatal()
        .write(|w| unsafe { w.bits(0xffff) });
    host.int_msk_data_id().write(|w| unsafe { w.bits(0xffff) });
    host.int_msk_ecc_corrected()
        .write(|w| unsafe { w.bits(0xffff) });
    host.int_msk_phy().write(|w| unsafe { w.bits(0x30003) });
}

fn set_sc2336_test_pattern(
    i2c: &mut I2c<'_, Blocking>,
    enable: bool,
) -> Result<u8, i2c::master::Error> {
    let value = read_reg_u8(i2c, SC2336_REG_TEST_PATTERN)?;
    let value = if enable {
        value | SC2336_TEST_PATTERN_BIT
    } else {
        value & !SC2336_TEST_PATTERN_BIT
    };
    write_reg_u8(i2c, SC2336_REG_TEST_PATTERN, value)?;
    read_reg_u8(i2c, SC2336_REG_TEST_PATTERN)
}

// Register names, encodings and limits from esp-cam-sensor `sc2336_regs.h` and
// `sc2336.c` (`sc2336_set_exp_val`, `sc2336_set_total_gain_val`).
const SC2336_REG_SHUTTER_TIME_H: u16 = 0x3e00;
const SC2336_REG_SHUTTER_TIME_M: u16 = 0x3e01;
const SC2336_REG_SHUTTER_TIME_L: u16 = 0x3e02;
const SC2336_REG_DIG_COARSE_GAIN: u16 = 0x3e06;
const SC2336_REG_DIG_FINE_GAIN: u16 = 0x3e07;
const SC2336_REG_ANG_GAIN: u16 = 0x3e09;
// `isp_v1_info` of `MIPI_2lane_24Minput_RAW8_1024x600_30fps`.
const SC2336_MODE_VTS: u32 = 1000;
const SC2336_MODE_TLINE_NS: u32 = 33_333;
const SC2336_EXPOSURE_MIN_LINES: u32 = 1;
// `SC2336_EXP_MAX_OFFSET`.
const SC2336_EXPOSURE_MAX_LINES: u32 = SC2336_MODE_VTS - 6;
// Last entry not above `CONFIG_CAMERA_SC2336_ABSOLUTE_GAIN_LIMIT` (default 66016),
// the same as the driver's `s_limited_gain_index`.
const SC2336_GAIN_INDEX_LIMIT: usize = 193;

// esp-cam-sensor `sc2336.c`, `CONFIG_CAMERA_SC2336_DIG_GAIN_PRIORITY` (the Kconfig default):
// (total gain x1000, 0x3e07 digital fine, 0x3e06 digital coarse, 0x3e09 analog).
const SC2336_GAIN_MAP: [(u32, u8, u8, u8); 224] = [
    (1000, 0x80, 0x00, 0x00),
    (1031, 0x84, 0x00, 0x00),
    (1063, 0x88, 0x00, 0x00),
    (1094, 0x8c, 0x00, 0x00),
    (1125, 0x90, 0x00, 0x00),
    (1156, 0x94, 0x00, 0x00),
    (1188, 0x98, 0x00, 0x00),
    (1219, 0x9c, 0x00, 0x00),
    (1250, 0xa0, 0x00, 0x00),
    (1281, 0xa4, 0x00, 0x00),
    (1313, 0xa8, 0x00, 0x00),
    (1344, 0xac, 0x00, 0x00),
    (1375, 0xb0, 0x00, 0x00),
    (1406, 0xb4, 0x00, 0x00),
    (1438, 0xb8, 0x00, 0x00),
    (1469, 0xbc, 0x00, 0x00),
    (1500, 0xc0, 0x00, 0x00),
    (1531, 0xc4, 0x00, 0x00),
    (1563, 0xc8, 0x00, 0x00),
    (1594, 0xcc, 0x00, 0x00),
    (1625, 0xd0, 0x00, 0x00),
    (1656, 0xd4, 0x00, 0x00),
    (1688, 0xd8, 0x00, 0x00),
    (1719, 0xdc, 0x00, 0x00),
    (1750, 0xe0, 0x00, 0x00),
    (1781, 0xe4, 0x00, 0x00),
    (1813, 0xe8, 0x00, 0x00),
    (1844, 0xec, 0x00, 0x00),
    (1875, 0xf0, 0x00, 0x00),
    (1906, 0xf4, 0x00, 0x00),
    (1938, 0xf8, 0x00, 0x00),
    (1969, 0xfc, 0x00, 0x00),
    (2000, 0x80, 0x01, 0x00),
    (2063, 0x84, 0x01, 0x00),
    (2125, 0x88, 0x01, 0x00),
    (2188, 0x8c, 0x01, 0x00),
    (2250, 0x90, 0x01, 0x00),
    (2313, 0x94, 0x01, 0x00),
    (2375, 0x98, 0x01, 0x00),
    (2438, 0x9c, 0x01, 0x00),
    (2500, 0xa0, 0x01, 0x00),
    (2563, 0xa4, 0x01, 0x00),
    (2625, 0xa8, 0x01, 0x00),
    (2688, 0xac, 0x01, 0x00),
    (2750, 0xb0, 0x01, 0x00),
    (2813, 0xb4, 0x01, 0x00),
    (2875, 0xb8, 0x01, 0x00),
    (2938, 0xbc, 0x01, 0x00),
    (3000, 0xc0, 0x01, 0x00),
    (3063, 0xc4, 0x01, 0x00),
    (3125, 0xc8, 0x01, 0x00),
    (3188, 0xcc, 0x01, 0x00),
    (3250, 0xd0, 0x01, 0x00),
    (3313, 0xd4, 0x01, 0x00),
    (3375, 0xd8, 0x01, 0x00),
    (3438, 0xdc, 0x01, 0x00),
    (3500, 0xe0, 0x01, 0x00),
    (3563, 0xe4, 0x01, 0x00),
    (3625, 0xe8, 0x01, 0x00),
    (3688, 0xec, 0x01, 0x00),
    (3750, 0xf0, 0x01, 0x00),
    (3813, 0xf4, 0x01, 0x00),
    (3875, 0xf8, 0x01, 0x00),
    (3938, 0xfc, 0x01, 0x00),
    (4000, 0x80, 0x01, 0x08),
    (4126, 0x84, 0x01, 0x08),
    (4250, 0x88, 0x01, 0x08),
    (4376, 0x8c, 0x01, 0x08),
    (4500, 0x90, 0x01, 0x08),
    (4626, 0x94, 0x01, 0x08),
    (4750, 0x98, 0x01, 0x08),
    (4876, 0x9c, 0x01, 0x08),
    (5000, 0xa0, 0x01, 0x08),
    (5126, 0xa4, 0x01, 0x08),
    (5250, 0xa8, 0x01, 0x08),
    (5376, 0xac, 0x01, 0x08),
    (5500, 0xb0, 0x01, 0x08),
    (5626, 0xb4, 0x01, 0x08),
    (5750, 0xb8, 0x01, 0x08),
    (5876, 0xbc, 0x01, 0x08),
    (6000, 0xc0, 0x01, 0x08),
    (6126, 0xc4, 0x01, 0x08),
    (6250, 0xc8, 0x01, 0x08),
    (6376, 0xcc, 0x01, 0x08),
    (6500, 0xd0, 0x01, 0x08),
    (6626, 0xd4, 0x01, 0x08),
    (6750, 0xd8, 0x01, 0x08),
    (6876, 0xdc, 0x01, 0x08),
    (7000, 0xe0, 0x01, 0x08),
    (7126, 0xe4, 0x01, 0x08),
    (7250, 0xe8, 0x01, 0x08),
    (7376, 0xec, 0x01, 0x08),
    (7500, 0xf0, 0x01, 0x08),
    (7626, 0xf4, 0x01, 0x08),
    (7750, 0xf8, 0x01, 0x08),
    (7876, 0xfc, 0x01, 0x08),
    (8000, 0x80, 0x01, 0x09),
    (8252, 0x84, 0x01, 0x09),
    (8500, 0x88, 0x01, 0x09),
    (8752, 0x8c, 0x01, 0x09),
    (9000, 0x90, 0x01, 0x09),
    (9252, 0x94, 0x01, 0x09),
    (9500, 0x98, 0x01, 0x09),
    (9752, 0x9c, 0x01, 0x09),
    (10000, 0xa0, 0x01, 0x09),
    (10252, 0xa4, 0x01, 0x09),
    (10500, 0xa8, 0x01, 0x09),
    (10752, 0xac, 0x01, 0x09),
    (11000, 0xb0, 0x01, 0x09),
    (11252, 0xb4, 0x01, 0x09),
    (11500, 0xb8, 0x01, 0x09),
    (11752, 0xbc, 0x01, 0x09),
    (12000, 0xc0, 0x01, 0x09),
    (12252, 0xc4, 0x01, 0x09),
    (12500, 0xc8, 0x01, 0x09),
    (12752, 0xcc, 0x01, 0x09),
    (13000, 0xd0, 0x01, 0x09),
    (13252, 0xd4, 0x01, 0x09),
    (13500, 0xd8, 0x01, 0x09),
    (13752, 0xdc, 0x01, 0x09),
    (14000, 0xe0, 0x01, 0x09),
    (14252, 0xe4, 0x01, 0x09),
    (14500, 0xe8, 0x01, 0x09),
    (14752, 0xec, 0x01, 0x09),
    (15000, 0xf0, 0x01, 0x09),
    (15252, 0xf4, 0x01, 0x09),
    (15500, 0xf8, 0x01, 0x09),
    (15752, 0xfc, 0x01, 0x09),
    (16000, 0x80, 0x01, 0x0b),
    (16504, 0x84, 0x01, 0x0b),
    (17000, 0x88, 0x01, 0x0b),
    (17504, 0x8c, 0x01, 0x0b),
    (18000, 0x90, 0x01, 0x0b),
    (18504, 0x94, 0x01, 0x0b),
    (19000, 0x98, 0x01, 0x0b),
    (19504, 0x9c, 0x01, 0x0b),
    (20000, 0xa0, 0x01, 0x0b),
    (20504, 0xa4, 0x01, 0x0b),
    (21000, 0xa8, 0x01, 0x0b),
    (21504, 0xac, 0x01, 0x0b),
    (22000, 0xb0, 0x01, 0x0b),
    (22504, 0xb4, 0x01, 0x0b),
    (23000, 0xb8, 0x01, 0x0b),
    (23504, 0xbc, 0x01, 0x0b),
    (24000, 0xc0, 0x01, 0x0b),
    (24504, 0xc4, 0x01, 0x0b),
    (25000, 0xc8, 0x01, 0x0b),
    (25504, 0xcc, 0x01, 0x0b),
    (26000, 0xd0, 0x01, 0x0b),
    (26504, 0xd4, 0x01, 0x0b),
    (27000, 0xd8, 0x01, 0x0b),
    (27504, 0xdc, 0x01, 0x0b),
    (28000, 0xe0, 0x01, 0x0b),
    (28504, 0xe4, 0x01, 0x0b),
    (29000, 0xe8, 0x01, 0x0b),
    (29504, 0xec, 0x01, 0x0b),
    (30000, 0xf0, 0x01, 0x0b),
    (30504, 0xf4, 0x01, 0x0b),
    (31000, 0xf8, 0x01, 0x0b),
    (31504, 0xfc, 0x01, 0x0b),
    (32000, 0x80, 0x01, 0x0f),
    (33008, 0x84, 0x01, 0x0f),
    (34000, 0x88, 0x01, 0x0f),
    (35008, 0x8c, 0x01, 0x0f),
    (36000, 0x90, 0x01, 0x0f),
    (37008, 0x94, 0x01, 0x0f),
    (38000, 0x98, 0x01, 0x0f),
    (39008, 0x9c, 0x01, 0x0f),
    (40000, 0xa0, 0x01, 0x0f),
    (41008, 0xa4, 0x01, 0x0f),
    (42000, 0xa8, 0x01, 0x0f),
    (43008, 0xac, 0x01, 0x0f),
    (44000, 0xb0, 0x01, 0x0f),
    (45008, 0xb4, 0x01, 0x0f),
    (46000, 0xb8, 0x01, 0x0f),
    (47008, 0xbc, 0x01, 0x0f),
    (48000, 0xc0, 0x01, 0x0f),
    (49008, 0xc4, 0x01, 0x0f),
    (50000, 0xc8, 0x01, 0x0f),
    (51008, 0xcc, 0x01, 0x0f),
    (52000, 0xd0, 0x01, 0x0f),
    (53008, 0xd4, 0x01, 0x0f),
    (54000, 0xd8, 0x01, 0x0f),
    (55008, 0xdc, 0x01, 0x0f),
    (56000, 0xe0, 0x01, 0x0f),
    (57008, 0xe4, 0x01, 0x0f),
    (58000, 0xe8, 0x01, 0x0f),
    (59008, 0xec, 0x01, 0x0f),
    (60000, 0xf0, 0x01, 0x0f),
    (61008, 0xf4, 0x01, 0x0f),
    (62000, 0xf8, 0x01, 0x0f),
    (63008, 0xfc, 0x01, 0x0f),
    (64000, 0x80, 0x01, 0x1f),
    (66016, 0x84, 0x01, 0x1f),
    (68000, 0x88, 0x01, 0x1f),
    (70016, 0x8c, 0x01, 0x1f),
    (72000, 0x90, 0x01, 0x1f),
    (74016, 0x94, 0x01, 0x1f),
    (76000, 0x98, 0x01, 0x1f),
    (78016, 0x9c, 0x01, 0x1f),
    (80000, 0xa0, 0x01, 0x1f),
    (82016, 0xa4, 0x01, 0x1f),
    (84000, 0xa8, 0x01, 0x1f),
    (86016, 0xac, 0x01, 0x1f),
    (88000, 0xb0, 0x01, 0x1f),
    (90016, 0xb4, 0x01, 0x1f),
    (92000, 0xb8, 0x01, 0x1f),
    (94016, 0xbc, 0x01, 0x1f),
    (96000, 0xc0, 0x01, 0x1f),
    (98016, 0xc4, 0x01, 0x1f),
    (100000, 0xc8, 0x01, 0x1f),
    (102016, 0xcc, 0x01, 0x1f),
    (104000, 0xd0, 0x01, 0x1f),
    (106016, 0xd4, 0x01, 0x1f),
    (108000, 0xd8, 0x01, 0x1f),
    (110016, 0xdc, 0x01, 0x1f),
    (112000, 0xe0, 0x01, 0x1f),
    (114016, 0xe4, 0x01, 0x1f),
    (116000, 0xe8, 0x01, 0x1f),
    (118016, 0xec, 0x01, 0x1f),
    (120000, 0xf0, 0x01, 0x1f),
    (122016, 0xf4, 0x01, 0x1f),
    (124000, 0xf8, 0x01, 0x1f),
    (126016, 0xfc, 0x01, 0x1f),
];

fn env_number(name: &str, value: Option<&str>) -> Option<u32> {
    value.map(|text| {
        text.parse()
            .unwrap_or_else(|_| panic!("{name} must be a decimal number, got '{text}'"))
    })
}

/// Writes the exposure and gain overrides while the sensor is in standby, in
/// the register order of the official driver.
fn apply_sc2336_exposure_gain(
    i2c: &mut I2c<'_, Blocking>,
    exposure_lines: Option<u32>,
    gain_index: Option<u32>,
) -> Result<(), i2c::master::Error> {
    if let Some(lines) = exposure_lines {
        assert!(
            (SC2336_EXPOSURE_MIN_LINES..=SC2336_EXPOSURE_MAX_LINES).contains(&lines),
            "CSI_EXPOSURE_LINES must be {SC2336_EXPOSURE_MIN_LINES}..={SC2336_EXPOSURE_MAX_LINES}, got {lines}"
        );
        // The low 4 bits of 0x3e02 are a fraction of a line and stay 0.
        write_reg_u8(i2c, SC2336_REG_SHUTTER_TIME_H, ((lines >> 12) & 0xf) as u8)?;
        write_reg_u8(i2c, SC2336_REG_SHUTTER_TIME_M, ((lines >> 4) & 0xff) as u8)?;
        write_reg_u8(i2c, SC2336_REG_SHUTTER_TIME_L, ((lines & 0xf) << 4) as u8)?;
        println!(
            "SC2336 exposure override: requested={} lines ({} us)",
            lines,
            lines * SC2336_MODE_TLINE_NS / 1000
        );
    }
    if let Some(index) = gain_index {
        assert!(
            index as usize <= SC2336_GAIN_INDEX_LIMIT,
            "CSI_GAIN_INDEX must be 0..={SC2336_GAIN_INDEX_LIMIT}, got {index}"
        );
        let (total, fine, coarse, analog) = SC2336_GAIN_MAP[index as usize];
        write_reg_u8(i2c, SC2336_REG_DIG_FINE_GAIN, fine)?;
        write_reg_u8(i2c, SC2336_REG_DIG_COARSE_GAIN, coarse)?;
        write_reg_u8(i2c, SC2336_REG_ANG_GAIN, analog)?;
        println!(
            "SC2336 gain override: requested index={} total_gain_x1000={} 0x3e07=0x{:02x} 0x3e06=0x{:02x} 0x3e09=0x{:02x}",
            index, total, fine, coarse, analog
        );
    }
    Ok(())
}

fn print_sc2336_exposure_gain(i2c: &mut I2c<'_, Blocking>) -> Result<(), i2c::master::Error> {
    let high = read_reg_u8(i2c, SC2336_REG_SHUTTER_TIME_H)?;
    let mid = read_reg_u8(i2c, SC2336_REG_SHUTTER_TIME_M)?;
    let low = read_reg_u8(i2c, SC2336_REG_SHUTTER_TIME_L)?;
    let lines = ((high as u32 & 0xf) << 12) | ((mid as u32) << 4) | (low as u32 >> 4);
    println!(
        "SC2336 exposure readback: 0x3e00=0x{:02x} 0x3e01=0x{:02x} 0x3e02=0x{:02x} lines={} ({} us of a {} us frame, max {} lines)",
        high,
        mid,
        low,
        lines,
        lines * SC2336_MODE_TLINE_NS / 1000,
        SC2336_MODE_VTS * SC2336_MODE_TLINE_NS / 1000,
        SC2336_EXPOSURE_MAX_LINES
    );

    let fine = read_reg_u8(i2c, SC2336_REG_DIG_FINE_GAIN)?;
    let coarse = read_reg_u8(i2c, SC2336_REG_DIG_COARSE_GAIN)?;
    let analog = read_reg_u8(i2c, SC2336_REG_ANG_GAIN)?;
    print!(
        "SC2336 gain readback: 0x3e07=0x{:02x} 0x3e06=0x{:02x} 0x3e09=0x{:02x}",
        fine, coarse, analog
    );
    match SC2336_GAIN_MAP
        .iter()
        .position(|&(_, f, c, a)| (f, c, a) == (fine, coarse, analog))
    {
        Some(index) => println!(
            " index={} total_gain_x1000={}",
            index, SC2336_GAIN_MAP[index].0
        ),
        None => println!(" (not an entry of the official gain map)"),
    }
    Ok(())
}

fn print_bridge_state_transition(
    before: &CaptureDebugSnapshot,
    after: &CaptureDebugSnapshot,
    label: &str,
) {
    let mut changed = false;
    for (name, before_raw, after_raw) in [
        ("csi_en", before.bridge_csi_en_raw, after.bridge_csi_en_raw),
        (
            "host_ctrl",
            before.bridge_host_cm_ctrl_raw,
            after.bridge_host_cm_ctrl_raw,
        ),
        (
            "frame_cfg",
            before.bridge_frame_cfg_raw,
            after.bridge_frame_cfg_raw,
        ),
        (
            "data_type_cfg",
            before.bridge_data_type_cfg_raw,
            after.bridge_data_type_cfg_raw,
        ),
        (
            "dma_req_cfg",
            before.bridge_dma_req_cfg_raw,
            after.bridge_dma_req_cfg_raw,
        ),
        (
            "dmablk_size",
            before.bridge_dmablk_size_raw,
            after.bridge_dmablk_size_raw,
        ),
    ] {
        if before_raw != after_raw {
            changed = true;
            println!(
                "bridge-state({}): {} changed 0x{:08x} -> 0x{:08x}",
                label, name, before_raw, after_raw
            );
        }
    }

    if !changed {
        println!("bridge-state({}): unchanged by ISP init sequence", label);
    }
}

fn sentinel_word(index: usize) -> u32 {
    (index as u32).wrapping_mul(0x9E37_79B1) ^ SENTINEL_SEED
}

fn fill_sentinel(region: &mut [u8]) {
    for (index, word) in region.chunks_exact_mut(4).enumerate() {
        word.copy_from_slice(&sentinel_word(index).to_le_bytes());
    }
}

fn read_word(region: &[u8], index: usize) -> u32 {
    assert!((index + 1) * 4 <= region.len());
    // Volatile, because the DMA wrote this memory outside the compiler's view.
    unsafe { core::ptr::read_volatile(region.as_ptr().cast::<u32>().add(index)) }
}

#[derive(Clone, Copy)]
struct SentinelScan {
    written_words: usize,
    untouched_words: usize,
    first_untouched_byte: Option<usize>,
    written_end_byte: usize,
}

/// Compares `bytes` of `region` against the pattern of `fill_sentinel`.
fn scan_sentinel(region: &[u8], bytes: core::ops::Range<usize>) -> SentinelScan {
    let mut scan = SentinelScan {
        written_words: 0,
        untouched_words: 0,
        first_untouched_byte: None,
        written_end_byte: 0,
    };
    for index in bytes.start / 4..bytes.end / 4 {
        if read_word(region, index) == sentinel_word(index) {
            scan.untouched_words += 1;
            scan.first_untouched_byte.get_or_insert(index * 4);
        } else {
            scan.written_words += 1;
            scan.written_end_byte = (index + 1) * 4;
        }
    }
    scan
}

fn allocate_capture_buffer(mode: CaptureMode) -> &'static mut [u8] {
    let len = mode.expected_bytes() + GUARD_BYTES;
    let layout = Layout::from_size_align(len, PSRAM_DMA_ALIGNMENT).unwrap();
    let ptr = unsafe { alloc::alloc::alloc(layout) };
    if ptr.is_null() {
        panic!("frame allocation failed");
    }
    unsafe { core::slice::from_raw_parts_mut(ptr, len) }
}

fn print_capture_plan(mode: CaptureMode, snapshot: &CaptureDebugSnapshot, allocation: &[u8]) {
    let bridge_block_bursts = (snapshot.bridge_dmablk_size_raw & 0x1fff) as usize;
    println!(
        "plan({}): output={} expected_bytes={} dma_block_ts={} units ({} bytes) bridge_dmablk_size={} bursts ({} bytes) buffer=0x{:08x}+{} guard={} bytes",
        mode.label(),
        if mode.isp_enabled() {
            "RGB565 (ISP on)"
        } else {
            "RAW8 (ISP bypass)"
        },
        mode.expected_bytes(),
        mode.dma_block_bytes() / 8,
        mode.dma_block_bytes(),
        bridge_block_bursts,
        bridge_block_bursts * BRIDGE_DMA_BURST_BYTES,
        allocation.as_ptr() as u32,
        mode.expected_bytes(),
        GUARD_BYTES
    );
}

fn print_dma_block_registers(label: &str, snapshot: &CaptureDebugSnapshot) {
    let block_ts_units = snapshot.dma_block_ts0_raw as usize + 1;
    let bridge_block_bursts = (snapshot.bridge_dmablk_size_raw & 0x1fff) as usize;
    println!(
        "{label}: programmed BLOCK_TS={} units ({} bytes) bridge DMABLK_SIZE={} bursts ({} bytes) SAR=0x{:08x} DAR=0x{:08x}",
        block_ts_units,
        block_ts_units * 8,
        bridge_block_bursts,
        bridge_block_bursts * BRIDGE_DMA_BURST_BYTES,
        snapshot.dma_sar0_raw,
        snapshot.dma_dar0_raw
    );
}

fn print_receive_error(label: &str, err: &ReceiveError) {
    match err {
        ReceiveError::Timeout(status)
        | ReceiveError::DmaError(status)
        | ReceiveError::CsiError(status) => print_capture_status("status", status),
        ReceiveError::ShutdownFailed {
            reason,
            status_before_shutdown,
            status_after_shutdown,
        } => {
            println!("{label}: shutdown failure reason: {:?}", reason);
            print_capture_status("status-before-shutdown", status_before_shutdown);
            print_capture_status("status-after-shutdown", status_after_shutdown);
        }
        ReceiveError::InvalidBuffer(buffer_err) => {
            println!("{label}: buffer error detail: {:?}", buffer_err);
        }
        ReceiveError::BufferTooSmall { required, provided } => {
            println!("{label}: buffer too small: required={required} provided={provided}");
        }
        _ => println!("{label}: unhandled receive error variant"),
    }
}

fn run_capture(
    capture: &mut CsiCapture<'_>,
    i2c: &mut I2c<'_, Blocking>,
    mode: CaptureMode,
    allocation: &mut [u8],
    test_pattern: bool,
    frame_export: bool,
    completion: Completion,
) -> bool {
    let label = mode.label();
    let capture_len = mode.expected_bytes();

    let _ = print_isp_input_gate_readback("before-arm");
    let before_arm = debug_snapshot();
    print_capture_debug_snapshot("before-arm", &before_arm);
    print_dma_axi_readback("before-arm", &before_arm);
    clear_isp_interrupts();

    fill_sentinel(allocation);
    DmaAlignedMut::new(&mut *allocation)
        .expect("capture allocation is DMA aligned")
        .writeback();

    let mut stream_on = None;
    let result = receive_frame(capture, completion, &mut allocation[..capture_len], || {
        stream_on = Some(set_sc2336_stream(i2c, true));
    });
    // Before the next frame reaches the stopped bridge and can set ISP
    // overflow bits that have nothing to do with the captured frame.
    let isp_int_raw = read_mmio_u32(ISP_INT_RAW_REG_ADDR);
    let stream_off = set_sc2336_stream(i2c, false);
    DmaAlignedMut::new(&mut *allocation)
        .expect("capture allocation is DMA aligned")
        .invalidate();

    println!(
        "{label}: stream_on={:?} stream_off={:?}",
        stream_on, stream_off
    );
    let after = debug_snapshot();
    print_capture_debug_snapshot("after-shutdown", &after);
    print_dma_axi_readback("after-shutdown", &after);
    print_dma_block_registers(label, &after);
    print!("{label}: isp int_raw at receive return=0x{isp_int_raw:08x}");
    print_bit_names(isp_int_raw, ISP_INT_NAMES);
    println!();
    let _ = print_isp_input_gate_readback("after-capture");
    print_host_detail_status("after-capture");
    if let Err(err) = &result {
        print_receive_error(label, err);
    }

    let valid = print_verdict(
        label,
        &CaptureEvidence {
            mode,
            result: &result,
            isp_int_raw,
            allocation,
            test_pattern,
        },
    ) && completion_interrupt_ok(label, capture, completion, true);

    if frame_export {
        // `receive` returning `Ok` means the DMA block completed and the
        // channel was shown stopped. The buffer was invalidated after that,
        // and `valid` includes the length, sentinel, and guard checks.
        match (mode, &result) {
            (CaptureMode::Raw8Psram, Ok(_)) if valid => {
                export_raw8_frame(i2c, &allocation[..capture_len], test_pattern)
            }
            (CaptureMode::Raw8Psram, _) => {
                println!("{label}: frame export skipped, the capture is not a valid frame")
            }
            _ => println!("{label}: frame export skipped, only mode A exports a frame"),
        }
    }

    valid
}

const MAX_REPEAT_FRAMES: u32 = 1000;

/// Buffer, guard, buffer, guard, from one PSRAM allocation.
fn allocate_stream_buffers(
    frame_bytes: usize,
) -> (
    &'static mut [u8],
    &'static mut [u8],
    &'static mut [u8],
    &'static mut [u8],
) {
    // One region for every caller: each stops its stream and drops the
    // buffers before the next one allocates, so `CSI_FAULT_ROUNDS` does not
    // run out of PSRAM.
    static REGION: core::sync::atomic::AtomicPtr<u8> =
        core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());
    let len = 2 * (frame_bytes + GUARD_BYTES);
    let mut ptr = REGION.load(core::sync::atomic::Ordering::Relaxed);
    if ptr.is_null() {
        let layout = Layout::from_size_align(len, PSRAM_DMA_ALIGNMENT).unwrap();
        ptr = unsafe { alloc::alloc::alloc(layout) };
        if ptr.is_null() {
            panic!("stream buffer allocation failed");
        }
        REGION.store(ptr, core::sync::atomic::Ordering::Relaxed);
    }
    let region = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
    let (first, second) = region.split_at_mut(frame_bytes + GUARD_BYTES);
    let (buffer0, guard0) = first.split_at_mut(frame_bytes);
    let (buffer1, guard1) = second.split_at_mut(frame_bytes);
    (buffer0, guard0, buffer1, guard1)
}

fn sentinel_writeback(region: &mut [u8]) {
    fill_sentinel(region);
    DmaAlignedMut::new(region)
        .expect("stream region is DMA aligned")
        .writeback();
}

fn guard_written_words(guard: &mut [u8]) -> usize {
    DmaAlignedMut::new(&mut *guard)
        .expect("guard is DMA aligned")
        .invalidate();
    scan_sentinel(guard, 0..guard.len()).written_words
}

/// While streaming, the SC2336 test pattern varies by one LSB in part of the
/// image. Frames match exactly only when each follows a fresh stream on.
const STREAMING_TEST_PATTERN_MAX_DELTA: u8 = 1;

/// Where `frame` differs from `reference`, as rows and columns of the RAW8
/// image. Returns the largest per-byte difference.
fn frame_difference(index: u32, sequence: u32, reference: &[u8], frame: &[u8], print: bool) -> u8 {
    let width = SC2336_FRAME_WIDTH as usize;
    let mut differing = 0usize;
    let mut first = None;
    let mut last = 0;
    let mut rows = (usize::MAX, 0usize);
    let mut cols = (usize::MAX, 0usize);
    let mut max_delta = 0u8;
    for (offset, (a, b)) in reference.iter().zip(frame).enumerate() {
        if a != b {
            differing += 1;
            first.get_or_insert(offset);
            last = offset;
            let (row, col) = (offset / width, offset % width);
            rows = (rows.0.min(row), rows.1.max(row));
            cols = (cols.0.min(col), cols.1.max(col));
            max_delta = max_delta.max(a.abs_diff(*b));
        }
    }
    if print || max_delta > STREAMING_TEST_PATTERN_MAX_DELTA {
        println!(
            "stream[{index}]: sequence={sequence} differs_from_first: bytes={differing} first_offset={first:?} last_offset={last} rows={}..={} cols={}..={} max_delta={max_delta}",
            rows.0, rows.1, cols.0, cols.1
        );
    }
    max_delta
}

/// Stop/start cycles in the `CSI_STREAM_FAULTS` start/stop case.
const STREAM_RESTART_CYCLES: u32 = 20;
/// How long the slow consumer holds a frame, about nine sensor frames.
const STREAM_HOLD_MS: u64 = 300;

const ISP_STATE_REG_ADDR: usize = 0x500A_126C;
/// `INT_RAW` bit of the real input frame end of `isp_input`.
const ISP_INT_INPUT_FRAME_END: u32 = 1 << 28;

/// One line of the state that decides whether frames reach the DMA after a
/// restart: sensor stream bit, D-PHY lanes, host, bridge, ISP and the
/// capture channel. Reads only, apart from clearing the ISP raw interrupts
/// when `clear_isp` is set, so the next snapshot shows new input frames.
fn restart_snapshot(label: &str, i2c: &mut I2c<'_, Blocking>, channel: usize, clear_isp: bool) {
    let sensor = read_reg_u8(i2c, SC2336_REG_STREAM_CTRL);
    let host = MIPI_CSI_HOST::regs();
    let bridge = MIPI_CSI_BRIDGE::regs();
    let clkrst = HP_SYS_CLKRST::regs();
    let isp_clock =
        clkrst.peri_clk_ctrl25().read().bits() & HP_SYS_CLKRST_PERI_CLK_CTRL25_ISP_CLK_EN_BIT != 0;
    let isp_reset =
        clkrst.hp_rst_en0().read().bits() & HP_SYS_CLKRST_HP_RST_EN0_RST_EN_ISP_BIT != 0;
    let isp = if isp_clock && !isp_reset {
        let regs = [
            read_mmio_u32(ISP_CNTL_REG_ADDR),
            read_mmio_u32(ISP_FRAME_CFG_REG_ADDR),
            read_mmio_u32(ISP_INT_RAW_REG_ADDR),
            read_mmio_u32(ISP_STATE_REG_ADDR),
        ];
        if clear_isp {
            write_mmio_u32(ISP_INT_CLR_REG_ADDR, u32::MAX);
        }
        Some(regs)
    } else {
        None
    };
    let vdma = VDMA::regs();
    let ch = vdma.ch(channel);
    let buf = bridge.buf_flow_ctl().read().bits();
    println!(
        "restart-snap[{label}]: sensor_0100={sensor:?} phy_stopstate=0x{:x} phy_rx=0x{:x} host_int_st_main=0x{:x} bridge_en={} bridge_int_raw=0x{:x} bridge_fifo_count={} isp_clk={isp_clock} isp_rst={isp_reset} isp(cntl,frame_cfg,int_raw,state)={:x?} isp_input_frame_end={} chen0=0x{:x} sar=0x{:08x} dar=0x{:08x} block_ts=0x{:x} intstatus=0x{:x} intsignal=0x{:x}",
        host.phy_stopstate().read().bits(),
        host.phy_rx().read().bits(),
        host.int_st_main().read().bits(),
        bridge.csi_en().read().csi_brig_en().bit(),
        bridge.int_raw().read().bits(),
        (buf >> 16) & 0x3fff,
        isp,
        isp.is_some_and(|r| r[2] & ISP_INT_INPUT_FRAME_END != 0),
        vdma.chen0().read().bits() & 0xf,
        ch.sar0().read().bits(),
        ch.dar0().read().bits(),
        ch.block_ts0().read().bits(),
        ch.intstatus0().read().bits(),
        ch.intsignal_enable0().read().bits(),
    );
}

fn fault_case(name: &str, ok: bool, detail: String) -> bool {
    println!(
        "stream-fault[{name}]: {} {detail}",
        if ok { "PASS" } else { "FAIL" }
    );
    ok
}

fn buffers_untouched(buffers: &mut [&'static mut [u8]; 2]) -> bool {
    buffers
        .iter_mut()
        .all(|buffer| guard_written_words(buffer) == 0)
}

/// `CSI_STREAM_FAULTS=1`: bounded failure cases for `CsiStream`, starting with
/// the sensor stopped.
fn run_stream_fault_cases<'d>(
    capture: CsiCapture<'d>,
    i2c: &mut I2c<'_, Blocking>,
) -> (CsiCapture<'d>, bool) {
    let frame_bytes = CaptureMode::Raw8Psram.expected_bytes();
    let channel = CSI_DMA_CHANNEL;
    let (buffer0, guard0, buffer1, guard1) = allocate_stream_buffers(frame_bytes);
    for region in [&mut *buffer0, &mut *guard0, &mut *buffer1, &mut *guard1] {
        sentinel_writeback(region);
    }
    let mut capture = capture;
    let mut buffers = [buffer0, buffer1];
    let mut passed = true;
    let delay = Delay::new();
    restart_snapshot("Z-after-main-stream", i2c, channel, true);

    // No sensor frames: `next_frame` times out, and stopping a stream whose DMA
    // is armed but never started writing leaves both buffers untouched.
    let mut stream = match capture.into_stream(buffers) {
        Ok(stream) => stream,
        Err((capture, _, error)) => {
            println!("stream-fault: FAIL, did not start: {error:?}");
            return (capture, false);
        }
    };
    restart_snapshot("A-before-timeout", i2c, channel, false);
    let start = esp_hal::time::Instant::now();
    let result = stream
        .next_frame(Duration::from_millis(200))
        .map(|frame| frame.sequence());
    let waited_ms = start.elapsed().as_millis();
    let stats = stream.stats();
    (capture, buffers) = stream.stop();
    restart_snapshot("B-after-timeout-cleanup", i2c, channel, false);
    let untouched = buffers_untouched(&mut buffers);
    passed &= fault_case(
        "timeout",
        matches!(result, Err(StreamError::Timeout))
            && (200..=210).contains(&waited_ms)
            && stats.completed == 0
            && stats.errored == 0
            && untouched,
        format!(
            "result={result:?} waited_ms={waited_ms} completed={} errored={} buffers_untouched={untouched}",
            stats.completed, stats.errored
        ),
    );

    // Slow consumer: hold one frame while the sensor keeps streaming. The DMA
    // destination never enters the held buffer, and newer frames overwrite
    // each other in the other buffer.
    let mut stream = match capture.into_stream(buffers) {
        Ok(stream) => stream,
        Err((capture, _, error)) => {
            println!("stream-fault: FAIL, did not restart: {error:?}");
            return (capture, false);
        }
    };
    restart_snapshot("C-before-sensor-on", i2c, channel, true);
    let stream_on = set_sc2336_stream(i2c, true);
    delay.delay_millis(50);
    restart_snapshot("D-50ms-after-sensor-on", i2c, channel, true);
    // The first block can be the tail of the frame the previous stop cut.
    let warm_up = stream
        .next_frame(Duration::from_millis(500))
        .map(|frame| frame.sequence());
    restart_snapshot(
        if warm_up.is_ok() {
            "E-first-frame-ok"
        } else {
            "F-first-frame-timeout"
        },
        i2c,
        channel,
        false,
    );
    let before = stream.stats();
    let held = match stream.next_frame(Duration::from_millis(500)) {
        Ok(frame) => {
            let range = frame.as_ptr() as u32..frame.as_ptr() as u32 + frame.len() as u32;
            let sequence = frame.sequence();
            let hold_start = esp_hal::time::Instant::now();
            let (mut samples, mut inside) = (0u32, 0u32);
            while hold_start.elapsed() < Duration::from_millis(STREAM_HOLD_MS) {
                let dar = VDMA::regs().ch(channel).dar0().read().bits();
                samples += 1;
                inside += u32::from(range.contains(&dar));
            }
            drop(frame);
            Some((sequence, samples, inside))
        }
        Err(e) => {
            println!("stream-fault[slow-consumer]: first frame {e:?}");
            None
        }
    };
    let next = stream
        .next_frame(Duration::from_millis(500))
        .map(|frame| frame.sequence());
    let after = stream.stats();
    let overwritten = after.overwritten - before.overwritten;
    passed &= fault_case(
        "slow-consumer",
        stream_on.is_ok()
            && warm_up.is_ok()
            && held.is_some_and(|(_, samples, inside)| samples > 0 && inside == 0)
            && next.is_ok()
            && overwritten >= 5
            && after.errored == before.errored,
        format!(
            "held=(sequence, dar_samples, dar_inside_held)={held:?} next={next:?} overwritten_during_hold={overwritten} errored_during_hold={}",
            after.errored - before.errored
        ),
    );

    // Repeated start/stop with the sensor streaming. Each stop lands at a
    // different point of the frame, swept in 1.7 ms steps. The driver flushes
    // the bridge FIFO on every start, so the first block after a restart
    // should be whole; a short one is dropped and resynced.
    let mut cycles_ok = 0;
    let (mut errored, mut resyncs) = (0, 0);
    for cycle in 0..STREAM_RESTART_CYCLES {
        let stop_delay_us = cycle * 1_700 % 33_000;
        delay.delay_micros(stop_delay_us);
        if cycle > 0 {
            let stats = stream.stats();
            errored += stats.errored;
            resyncs += stats.resyncs;
        }
        (capture, buffers) = stream.stop();
        stream = match capture.into_stream(buffers) {
            Ok(stream) => stream,
            Err((capture, _, error)) => {
                println!("stream-fault[restart]: cycle {cycle} did not start: {error:?}");
                let _ = set_sc2336_stream(i2c, false);
                return (capture, false);
            }
        };
        let frames: Vec<_> = (0..2)
            .map(|_| {
                stream
                    .next_frame(Duration::from_millis(500))
                    .map(|frame| frame.sequence())
            })
            .collect();
        if frames.iter().all(Result::is_ok) {
            cycles_ok += 1;
        } else {
            println!(
                "stream-fault[restart]: cycle {cycle} stop_delay_us={stop_delay_us} frames={frames:?}"
            );
        }
    }
    let stats = stream.stats();
    errored += stats.errored;
    resyncs += stats.resyncs;
    let stream_off = set_sc2336_stream(i2c, false);
    (capture, buffers) = stream.stop();
    let _ = buffers;
    let guards = [guard_written_words(guard0), guard_written_words(guard1)];
    passed &= fault_case(
        "restart",
        cycles_ok == STREAM_RESTART_CYCLES
            && errored == resyncs
            && errored <= STREAM_RESTART_CYCLES
            && stream_off.is_ok()
            && guards == [0, 0],
        format!(
            "cycles_ok={cycles_ok}/{STREAM_RESTART_CYCLES} errored={errored} resyncs={resyncs} guard_written_words={guards:?}"
        ),
    );

    println!("stream-fault: {}", if passed { "PASS" } else { "FAIL" });
    (capture, passed)
}

/// `CSI_STREAM=1`: continuous capture with the sensor streaming throughout.
fn run_stream_capture<'d>(
    capture: CsiCapture<'d>,
    i2c: &mut I2c<'_, Blocking>,
    frames: u32,
    test_pattern: bool,
    quiet: bool,
    mut lcd: Option<&mut Lcd>,
) -> (CsiCapture<'d>, bool) {
    let frame_bytes = CaptureMode::Raw8Psram.expected_bytes();
    let (buffer0, guard0, buffer1, guard1) = allocate_stream_buffers(frame_bytes);
    for region in [&mut *buffer0, &mut *guard0, &mut *buffer1, &mut *guard1] {
        sentinel_writeback(region);
    }
    let addrs = [buffer0.as_ptr() as usize, buffer1.as_ptr() as usize];
    let lcd_overlap = lcd
        .as_deref()
        .is_some_and(|lcd| lcd.overlaps(buffer0) || lcd.overlaps(buffer1));
    println!(
        "stream: buffer0=0x{:08x}+{} buffer1=0x{:08x}+{} guard={} bytes each lcd_overlap={}",
        addrs[0], frame_bytes, addrs[1], frame_bytes, GUARD_BYTES, lcd_overlap
    );
    assert!(!lcd_overlap, "stream buffer overlaps a DSI frame buffer");

    let mut stream = match capture.into_stream([buffer0, buffer1]) {
        Ok(stream) => stream,
        Err((capture, _, error)) => {
            println!("stream: FAIL, did not start: {error:?}");
            return (capture, false);
        }
    };
    let stream_on = set_sc2336_stream(i2c, true);
    let lcd_irqs_before = lcd.as_deref().map(|lcd| lcd.dpi.dma_interrupt_count());

    let mut checksums = Vec::with_capacity(frames as usize);
    let mut first_fill_untouched = [None::<usize>; 2];
    let mut last_sequence = None::<u32>;
    let mut max_sequence_gap = 0;
    let mut error = None::<StreamError>;
    let mut reference = None::<Vec<u8>>;
    let mut pattern_max_delta = 0u8;
    let start = esp_hal::time::Instant::now();
    for index in 1..=frames {
        if let Some(lcd) = lcd.as_deref_mut() {
            lcd.render_step();
        }
        let frame = match stream.next_frame(Duration::from_millis(500)) {
            Ok(frame) => frame,
            Err(e) => {
                println!("stream[{index}/{frames}]: FAIL {e:?}");
                error = Some(e);
                break;
            }
        };
        let slot = usize::from(frame.as_ptr() as usize == addrs[1]);
        if first_fill_untouched[slot].is_none() {
            first_fill_untouched[slot] =
                Some(scan_sentinel(&frame, 0..frame_bytes).untouched_words);
        }
        let sequence = frame.sequence();
        if let Some(last) = last_sequence {
            max_sequence_gap = max_sequence_gap.max(sequence.wrapping_sub(last));
        }
        last_sequence = Some(sequence);
        let checksum = checksum32(&frame);
        if test_pattern {
            match &reference {
                None => reference = Some(frame.to_vec()),
                Some(reference) => {
                    let print = !quiet || index % 100 == 0;
                    let delta = frame_difference(index, sequence, reference, &frame, print);
                    pattern_max_delta = pattern_max_delta.max(delta);
                }
            }
        }
        drop(frame);
        checksums.push(checksum);
        if !quiet || index % 100 == 0 {
            // Not `{:08x}` with a `0x` prefix: the monitor resolves those as addresses.
            println!(
                "stream[{index}/{frames}]: sequence={sequence} buffer={slot} checksum32={checksum:08x} elapsed_ms={}",
                start.elapsed().as_millis()
            );
        }
    }
    let elapsed_ms = start.elapsed().as_millis().max(1);
    let stats = stream.stats();
    let stream_off = set_sc2336_stream(i2c, false);
    let (capture, [_, _]) = stream.stop();
    let guards = [guard_written_words(guard0), guard_written_words(guard1)];

    let lcd_progress = lcd.zip(lcd_irqs_before).map(|(lcd, before)| {
        (
            lcd.dpi.dma_interrupt_count().wrapping_sub(before),
            lcd.underrun_seen(),
        )
    });

    let mut distinct = checksums.clone();
    distinct.sort_unstable();
    distinct.dedup();
    println!(
        "stream: delivered={}/{} completed={} overwritten={} errored={} resyncs={} dma_irqs={} elapsed_ms={}",
        checksums.len(),
        frames,
        stats.completed,
        stats.overwritten,
        stats.errored,
        stats.resyncs,
        stats.interrupts,
        elapsed_ms
    );
    println!(
        "stream: completed_frames_per_s_x10={} delivered_frames_per_s_x10={} max_sequence_gap={} first_fill_untouched_words={:?} guard_written_words={:?} distinct_checksums={}{} sensor=continuous stream_on={:?} stream_off={:?}",
        stats.completed as u64 * 10_000 / elapsed_ms,
        checksums.len() as u64 * 10_000 / elapsed_ms,
        max_sequence_gap,
        first_fill_untouched,
        guards,
        distinct.len(),
        if test_pattern {
            format!(
                " test_pattern_max_delta={pattern_max_delta} (expected <= {STREAMING_TEST_PATTERN_MAX_DELTA})"
            )
        } else {
            String::new()
        },
        stream_on,
        stream_off
    );
    if let Some((irqs, underrun)) = lcd_progress {
        println!("stream: dsi_dma_irqs={irqs} dsi_underrun={underrun}");
    }

    let passed = error.is_none()
        && checksums.len() == frames as usize
        && stats.errored == 0
        && stats.resyncs == 0
        && first_fill_untouched
            .iter()
            .all(|untouched| untouched.is_none_or(|words| words == 0))
        && guards == [0, 0]
        && (!test_pattern || pattern_max_delta <= STREAMING_TEST_PATTERN_MAX_DELTA)
        && stream_on.is_ok()
        && stream_off.is_ok()
        && lcd_progress.is_none_or(|(irqs, underrun)| irqs > 0 && !underrun);
    println!("stream: {}", if passed { "PASS" } else { "FAIL" });
    (capture, passed)
}

static ASYNC_WOKEN: AtomicBool = AtomicBool::new(false);
static ASYNC_WAKES: AtomicU32 = AtomicU32::new(0);
/// Polls that returned `Pending` and were followed by a wake and a new poll.
static ASYNC_WOKEN_POLLS: AtomicU32 = AtomicU32::new(0);

fn flag_waker() -> Waker {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(core::ptr::null(), &VTABLE),
        |_| wake_flag(),
        |_| wake_flag(),
        |_| {},
    );
    fn wake_flag() {
        ASYNC_WAKES.fetch_add(1, Ordering::Relaxed);
        ASYNC_WOKEN.store(true, Ordering::Release);
    }
    // SAFETY: the vtable functions ignore the data pointer.
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
}

/// Polls `future` once, then again only after its waker fired, until it
/// completes or `timeout` passes. A future that never wakes times out.
fn poll_on_wake<F: Future>(future: core::pin::Pin<&mut F>, timeout: Duration) -> Option<F::Output> {
    let mut future = future;
    let waker = flag_waker();
    let mut cx = Context::from_waker(&waker);
    let deadline = esp_hal::time::Instant::now() + timeout;
    loop {
        ASYNC_WOKEN.store(false, Ordering::Release);
        if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return Some(output);
        }
        while !ASYNC_WOKEN.load(Ordering::Acquire) {
            if esp_hal::time::Instant::now() > deadline {
                return None;
            }
        }
        ASYNC_WOKEN_POLLS.fetch_add(1, Ordering::Relaxed);
    }
}

/// `CSI_ASYNC=1`: frames through `next_frame_async`, a dropped pending
/// future, a sensor restart, and a blocking frame after `into_blocking`.
fn run_async_stream_check<'d>(
    capture: CsiCapture<'d>,
    i2c: &mut I2c<'_, Blocking>,
    frames: u32,
    mut lcd: Option<&mut Lcd>,
) -> (CsiCapture<'d>, bool) {
    let frame_bytes = CaptureMode::Raw8Psram.expected_bytes();
    let (buffer0, guard0, buffer1, guard1) = allocate_stream_buffers(frame_bytes);
    for region in [&mut *buffer0, &mut *guard0, &mut *buffer1, &mut *guard1] {
        sentinel_writeback(region);
    }
    let mut stream = match capture.into_async().into_stream([buffer0, buffer1]) {
        Ok(stream) => stream,
        Err((capture, _, error)) => {
            println!("async: FAIL, did not start: {error:?}");
            return (capture.into_blocking(), false);
        }
    };
    let timeout = Duration::from_millis(500);
    let take = |stream: &mut esp_hal::mipi_csi::CsiStream<'d, esp_hal::Async>,
                label: &str|
     -> Option<(u32, usize)> {
        let future = pin!(stream.next_frame_async());
        match poll_on_wake(future, timeout) {
            Some(Ok(frame)) => Some((
                frame.sequence(),
                scan_sentinel(&frame, 0..frame_bytes).untouched_words,
            )),
            Some(Err(error)) => {
                println!("async: FAIL {label}: {error:?}");
                None
            }
            None => {
                println!("async: FAIL {label}: no wake within {timeout:?}");
                None
            }
        }
    };

    let stream_on = set_sc2336_stream(i2c, true);
    let wakes_before = ASYNC_WAKES.load(Ordering::Relaxed);
    let woken_polls_before = ASYNC_WOKEN_POLLS.load(Ordering::Relaxed);
    let start = esp_hal::time::Instant::now();
    let mut delivered = 0;
    let mut last_sequence = None::<u32>;
    let mut sequences_increase = true;
    let mut first_fill_untouched = 0;
    for index in 1..=frames {
        if let Some(lcd) = lcd.as_deref_mut() {
            lcd.render_step();
        }
        let Some((sequence, untouched)) = take(&mut stream, "frame") else {
            break;
        };
        if index <= 2 {
            first_fill_untouched += untouched;
        }
        sequences_increase &= last_sequence.is_none_or(|last| sequence > last);
        last_sequence = Some(sequence);
        delivered += 1;
    }
    let elapsed_ms = start.elapsed().as_millis().max(1);
    let wakes = ASYNC_WAKES
        .load(Ordering::Relaxed)
        .wrapping_sub(wakes_before);
    let woken_polls = ASYNC_WOKEN_POLLS
        .load(Ordering::Relaxed)
        .wrapping_sub(woken_polls_before);

    // The frame just taken was the newest, so a new future is pending until
    // the next frame completes. Dropping it must not lose that frame.
    let cancel_pending = {
        let future = pin!(stream.next_frame_async());
        let waker = flag_waker();
        future.poll(&mut Context::from_waker(&waker)).is_pending()
    };
    let after_cancel = (0..5)
        .filter(|_| take(&mut stream, "after-cancel").is_some())
        .count();

    // Restart through the stream while the sensor streams, as the fault cases
    // do. The first block can be the tail of the frame the stop cut.
    let (capture, buffers) = stream.stop();
    let mut stream = match capture.into_stream(buffers) {
        Ok(stream) => stream,
        Err((capture, _, error)) => {
            println!("async: FAIL, did not restart: {error:?}");
            let _ = set_sc2336_stream(i2c, false);
            return (capture.into_blocking(), false);
        }
    };
    let after_restart = (0..5)
        .filter(|_| take(&mut stream, "after-restart").is_some())
        .count();

    // Opt-in reproducer: stopping the sensor mid-frame while the stream stays
    // armed can leave the bridge FIFO full with no DMA interrupt, so no frame
    // completes until the stream is restarted.
    let sensor_restart = env_flag(option_env!("CSI_ASYNC_SENSOR_RESTART")).then(|| {
        let _ = set_sc2336_stream(i2c, false);
        Delay::new().delay_millis(100);
        let _ = set_sc2336_stream(i2c, true);
        let frames = (0..5)
            .filter(|_| take(&mut stream, "after-sensor-restart").is_some())
            .count();
        if frames < 5 {
            restart_snapshot("async-sensor-restart-stall", i2c, CSI_DMA_CHANNEL, false);
            print_capture_debug_snapshot("async-sensor-restart-stall", &debug_snapshot());
        }
        frames
    });
    let stream_off = set_sc2336_stream(i2c, false);
    let stats = stream.stats();
    let (capture, [buffer0, _]) = stream.stop();
    let guards = [guard_written_words(guard0), guard_written_words(guard1)];

    // The sensor stop above can cut a frame, and the host then reports an
    // error at the next frame start. One whole sensor frame before arming
    // lets that error latch before the capture, which clears it.
    let mut capture = capture.into_blocking();
    let settle_on = set_sc2336_stream(i2c, true);
    Delay::new().delay_millis(50);
    let blocking = capture
        .receive_with(&mut buffer0[..frame_bytes], || {})
        .map(|frame| frame.received_bytes);
    let settle_off = set_sc2336_stream(i2c, false);

    println!(
        "async: delivered={delivered}/{frames} wakes={wakes} woken_polls={woken_polls} delivered_frames_per_s_x10={} sequences_increase={sequences_increase} first_fill_untouched_words={first_fill_untouched} guard_written_words={guards:?}",
        delivered as u64 * 10_000 / elapsed_ms
    );
    println!(
        "async: cancel_pending={cancel_pending} after_cancel_frames={after_cancel}/5 after_restart_frames={after_restart}/5 sensor_restart_frames={sensor_restart:?} completed={} errored={} resyncs={} stream_on={stream_on:?} stream_off={stream_off:?}",
        stats.completed, stats.errored, stats.resyncs
    );
    println!(
        "async: blocking_after_into_blocking={blocking:?} sensor_on={settle_on:?} sensor_off={settle_off:?}"
    );
    // With the LCD, rendering between frames can take longer than a frame, so
    // every frame may be ready at the first poll.
    let passed = delivered == frames
        && (woken_polls > 0 || lcd.is_some())
        && sequences_increase
        && first_fill_untouched == 0
        && guards == [0, 0]
        && cancel_pending
        && after_cancel == 5
        && after_restart == 5
        && sensor_restart.is_none_or(|frames| frames == 5)
        && stream_on.is_ok()
        && stream_off.is_ok()
        && settle_on.is_ok()
        && settle_off.is_ok()
        && blocking.is_ok_and(|bytes| bytes == frame_bytes);
    println!("async: {}", if passed { "PASS" } else { "FAIL" });
    (capture, passed)
}

/// Captures `frames` RAW8 frames into the same PSRAM buffer, one full
/// single-shot capture per frame, and checks the buffer before it is reused.
/// The sensor streams only while a DMA block is armed, or throughout with
/// `continuous_sensor`.
#[expect(clippy::too_many_arguments)]
fn run_repeated_capture(
    capture: &mut CsiCapture<'_>,
    i2c: &mut I2c<'_, Blocking>,
    allocation: &mut [u8],
    test_pattern: bool,
    frames: u32,
    frame_export: bool,
    completion: Completion,
    mut lcd: Option<&mut Lcd>,
    continuous_sensor: bool,
    quiet: bool,
) -> bool {
    let mode = CaptureMode::Raw8Psram;
    let label = mode.label();
    let capture_len = mode.expected_bytes();

    let before_arm = debug_snapshot();
    print_capture_debug_snapshot("before-arm", &before_arm);
    print_dma_axi_readback("before-arm", &before_arm);

    let mut checksums = Vec::with_capacity(frames as usize);
    let continuous_stream_on = continuous_sensor.then(|| set_sc2336_stream(i2c, true));
    let mut last_done = None::<esp_hal::time::Instant>;
    let mut interval_ms = (u64::MAX, 0u64);
    let run_start = esp_hal::time::Instant::now();
    for frame in 1..=frames {
        let frame_start = esp_hal::time::Instant::now();
        clear_isp_interrupts();
        fill_sentinel(allocation);
        DmaAlignedMut::new(&mut *allocation)
            .expect("capture allocation is DMA aligned")
            .writeback();

        let lcd_before = lcd.as_deref_mut().map(Lcd::before_capture);
        let mut stream_on = continuous_stream_on;
        let result = receive_frame(capture, completion, &mut allocation[..capture_len], || {
            if !continuous_sensor {
                stream_on = Some(set_sc2336_stream(i2c, true));
            }
        });
        let done = esp_hal::time::Instant::now();
        if let Some(last) = last_done {
            let ms = (done - last).as_millis();
            interval_ms = (interval_ms.0.min(ms), interval_ms.1.max(ms));
        }
        last_done = Some(done);
        let lcd_progress = lcd
            .as_deref_mut()
            .zip(lcd_before)
            .map(|(lcd, before)| lcd.after_capture(before));
        let isp_int_raw = read_mmio_u32(ISP_INT_RAW_REG_ADDR);
        let stream_off = if continuous_sensor {
            Ok(())
        } else {
            set_sc2336_stream(i2c, false)
        };
        DmaAlignedMut::new(&mut *allocation)
            .expect("capture allocation is DMA aligned")
            .invalidate();

        let evidence = CaptureEvidence {
            mode,
            result: &result,
            isp_int_raw,
            allocation,
            test_pattern,
        };
        let assessment = assess(&evidence);
        let sensor_ok = matches!(stream_on, Some(Ok(_))) && stream_off.is_ok();
        let received = result.as_ref().map_or(0, |frame| frame.received_bytes);
        let irq_ok = completion_interrupt_ok(label, capture, completion, false);
        let lcd_ok = lcd_progress.is_none_or(|(irqs, _, _)| irqs > 0);
        let ok = assessment.valid && sensor_ok && received == capture_len && irq_ok && lcd_ok;
        let checksum = checksum32(&allocation[..capture_len]);

        if ok && quiet && frame % 100 != 0 {
            checksums.push(checksum);
            continue;
        }
        print!(
            "repeat[{}/{}]: ok={} receive={} received_bytes={} untouched_words={} guard_words_written={} bridge_errors=0x{:02x} host_error={} isp_errors={:#x} checksum32=",
            frame,
            frames,
            ok,
            if result.is_ok() { "Ok" } else { "Err" },
            received,
            assessment.capture_scan.untouched_words,
            assessment.guard_scan.written_words,
            assessment.bridge_errors,
            assessment.host_error,
            assessment.isp_errors,
        );
        // Not `{:08x}` with a `0x` prefix: the monitor resolves those as addresses.
        print!(
            "{:08x} frame_ms={}",
            checksum,
            frame_start.elapsed().as_millis()
        );
        if completion == Completion::Interrupt {
            print!(
                " irq_latency={:?}us",
                capture.completion_interrupt_report().latency_from_arm_us
            );
        }
        if let Some((irqs, rate_per_s, underrun)) = lcd_progress {
            print!(" dsi_dma_irqs={irqs} dsi_dma_irqs_per_s={rate_per_s} dsi_underrun={underrun}");
        }
        println!();

        if !ok {
            println!(
                "repeat[{frame}/{frames}]: FAIL stream_on={:?} stream_off={:?}, full diagnostics follow",
                stream_on, stream_off
            );
            let after = debug_snapshot();
            print_capture_debug_snapshot("after-shutdown", &after);
            print_dma_axi_readback("after-shutdown", &after);
            print_host_detail_status("after-capture");
            if let Err(err) = &result {
                print_receive_error(label, err);
            }
            print_verdict(label, &evidence);
            if continuous_sensor {
                let _ = set_sc2336_stream(i2c, false);
            }
            return false;
        }
        checksums.push(checksum);
    }
    let run_ms = run_start.elapsed().as_millis();
    let continuous_stream_off = continuous_sensor.then(|| set_sc2336_stream(i2c, false));

    let after = debug_snapshot();
    print_dma_axi_readback("after-shutdown", &after);
    let mut distinct = checksums.clone();
    distinct.sort_unstable();
    distinct.dedup();
    println!(
        "repeat: {}/{} frames valid in {} ms, distinct_checksums={}{}",
        checksums.len(),
        frames,
        run_ms,
        distinct.len(),
        if test_pattern && !continuous_sensor {
            " (test pattern: expected 1)"
        } else {
            ""
        }
    );
    println!(
        "repeat: sensor={} stream_off={:?} valid_frames_per_s_x10={} frame_interval_ms min={} max={}",
        if continuous_sensor {
            "continuous"
        } else {
            "restarted-per-frame"
        },
        continuous_stream_off,
        checksums.len() as u64 * 10_000 / run_ms.max(1),
        if frames > 1 { interval_ms.0 } else { 0 },
        interval_ms.1
    );
    if continuous_stream_off.is_some_and(|r| r.is_err()) {
        println!("repeat: FAIL, sensor stream did not stop");
        return false;
    }
    if test_pattern && !continuous_sensor && distinct.len() != 1 {
        println!("repeat: FAIL, test pattern frames differ");
        return false;
    }

    if frame_export {
        // The last frame passed the same checks as a single-shot export.
        println!("{label}: exporting frame {frames} of {frames}");
        export_raw8_frame(i2c, &allocation[..capture_len], test_pattern);
    }
    true
}

const EXPORT_PREFIX: &str = "@@SC2336FRAME";
const EXPORT_CHUNK_BYTES: usize = 512;
const EXPORT_BASE64_BYTES: usize = EXPORT_CHUNK_BYTES.div_ceil(3) * 4;
const EXPORT_FRAME_ID: u32 = 1;
const SC2336_REG_FLIP_MIRROR: u16 = 0x3221;
const BASE64_ALPHABET: &[u8; 64] =
    b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut index = 0;
    while index < 256 {
        let mut crc = index as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[index] = crc;
        index += 1;
    }
    table
};

/// CRC-32 (IEEE), the same as Python's `zlib.crc32`.
fn crc32_update(crc: u32, data: &[u8]) -> u32 {
    let mut crc = !crc;
    for &byte in data {
        crc = CRC32_TABLE[((crc ^ byte as u32) & 0xff) as usize] ^ (crc >> 8);
    }
    !crc
}

fn base64_encode<'a>(input: &[u8], output: &'a mut [u8; EXPORT_BASE64_BYTES]) -> &'a str {
    let mut len = 0;
    for group in input.chunks(3) {
        let b = [
            group[0],
            group.get(1).copied().unwrap_or(0),
            group.get(2).copied().unwrap_or(0),
        ];
        let n = (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32;
        let encoded = [
            BASE64_ALPHABET[(n >> 18) as usize & 63],
            BASE64_ALPHABET[(n >> 12) as usize & 63],
            if group.len() > 1 {
                BASE64_ALPHABET[(n >> 6) as usize & 63]
            } else {
                b'='
            },
            if group.len() > 2 {
                BASE64_ALPHABET[n as usize & 63]
            } else {
                b'='
            },
        ];
        output[len..len + 4].copy_from_slice(&encoded);
        len += 4;
    }
    core::str::from_utf8(&output[..len]).unwrap()
}

/// Prints `frame` as text lines that `examples/qa/tools/csi_frame_extract.py`
/// turns back into the raw frame.
///
/// espflash decodes the console as text, so the payload is base64. Numbers are
/// hex without a `0x` prefix, which espflash would try to resolve as symbols.
fn export_raw8_frame(i2c: &mut I2c<'_, Blocking>, frame: &[u8], test_pattern: bool) {
    // The Bayer order of the official mode table assumes mirror and flip are off.
    // `--` tells the host that the register could not be read.
    let flip_mirror = read_reg_u8(i2c, SC2336_REG_FLIP_MIRROR).map_or(u16::MAX, u16::from);
    let crc = crc32_update(0, frame);
    let chunks = frame.len().div_ceil(EXPORT_CHUNK_BYTES);
    print!("{EXPORT_PREFIX} BEGIN v=1 id={EXPORT_FRAME_ID} ");
    if flip_mirror == u16::MAX {
        print!("reg3221=-- ");
    } else {
        print!("reg3221={flip_mirror:02x} ");
    }
    println!(
        "width={SC2336_FRAME_WIDTH} height={SC2336_FRAME_HEIGHT} format=RAW8 bayer=BGGR bayer_source=esp-cam-sensor:MIPI_2lane_24Minput_RAW8_1024x600_30fps test_pattern={} buffer={:08x} len={} chunk_bytes={EXPORT_CHUNK_BYTES} chunks={chunks} crc32={crc:08x} checksum32={:08x}",
        test_pattern as u8,
        frame.as_ptr() as u32,
        frame.len(),
        checksum32(frame),
    );

    let mut encoded = [0u8; EXPORT_BASE64_BYTES];
    for (index, chunk) in frame.chunks(EXPORT_CHUNK_BYTES).enumerate() {
        println!(
            "{EXPORT_PREFIX} D {EXPORT_FRAME_ID} {index} {:x} {:08x} {}",
            index * EXPORT_CHUNK_BYTES,
            crc32_update(0, chunk),
            base64_encode(chunk, &mut encoded)
        );
    }

    println!(
        "{EXPORT_PREFIX} END id={EXPORT_FRAME_ID} len={} chunks={chunks} crc32={crc:08x}",
        frame.len()
    );
}

/// Prints how many rows equal the row two above, which share the same bayer
/// phase, and where row 0 changes value. Informational only.
fn print_test_pattern_rows(label: &str, frame: &[u8], bytes_per_pixel: usize) {
    let row_bytes = SC2336_FRAME_WIDTH as usize * bytes_per_pixel;
    let rows = frame.len() / row_bytes;
    let mut previous = [0u32; 2];
    let mut matching = 0;
    for row in 0..rows {
        let signature = checksum32(&frame[row * row_bytes..(row + 1) * row_bytes]);
        if row >= 2 && signature == previous[row % 2] {
            matching += 1;
        }
        previous[row % 2] = signature;
    }
    println!(
        "{label}: test-pattern rows equal to the row two above: {matching}/{}",
        rows.saturating_sub(2)
    );

    // RAW8 neighbours have different bayer colors, so RAW8 compares with the
    // pixel two to the left and RGB565 with the next one, both two bytes back.
    let stride = 2;
    print!("{label}: test-pattern row 0 changes at byte offsets:");
    let mut printed = 0;
    for offset in (stride..row_bytes).step_by(bytes_per_pixel) {
        if frame[offset..offset + bytes_per_pixel]
            != frame[offset - stride..offset - stride + bytes_per_pixel]
        {
            print!(" {offset}");
            printed += 1;
            if printed == 16 {
                break;
            }
        }
    }
    println!();
}

fn print_frame_stats(label: &str, frame: &[u8]) {
    let (mut min, mut max, mut sum) = (u8::MAX, 0u8, 0u64);
    for &byte in frame {
        min = min.min(byte);
        max = max.max(byte);
        sum += byte as u64;
    }
    println!(
        "{label}: frame bytes min={} max={} mean={} checksum32=0x{:08x}",
        min,
        max,
        sum / frame.len().max(1) as u64,
        checksum32(frame)
    );
    print_sample_bytes(frame, 32);
}

const BRIDGE_INT_ERROR_MASK: u32 = 0x1F;

struct CaptureEvidence<'a> {
    mode: CaptureMode,
    result: &'a Result<CaptureFrame, ReceiveError>,
    isp_int_raw: u32,
    allocation: &'a [u8],
    test_pattern: bool,
}

struct Assessment {
    written: usize,
    capture_scan: SentinelScan,
    guard_scan: SentinelScan,
    bytes_moved: bool,
    block_complete: bool,
    frame_sized: bool,
    bridge_errors: u32,
    host_error: bool,
    isp_errors: u32,
    valid: bool,
}

/// Applies the four result levels to one capture without printing.
fn assess(evidence: &CaptureEvidence<'_>) -> Assessment {
    let mode = evidence.mode;
    let expected = mode.expected_bytes();
    let error_status: Option<CaptureStatus> = match evidence.result {
        Ok(_) => None,
        Err(
            ReceiveError::Timeout(status)
            | ReceiveError::DmaError(status)
            | ReceiveError::CsiError(status),
        ) => Some(*status),
        Err(ReceiveError::ShutdownFailed {
            status_before_shutdown,
            ..
        }) => Some(*status_before_shutdown),
        Err(_) => None,
    };

    let capture_scan = scan_sentinel(evidence.allocation, 0..expected);
    let guard_scan = scan_sentinel(evidence.allocation, expected..evidence.allocation.len());

    // `received_bytes` is measured from the DMA destination address; after an
    // error only the sentinel shows how far the DMA wrote.
    let written = match evidence.result {
        Ok(frame) => frame.received_bytes,
        Err(_) => capture_scan.written_end_byte,
    };
    let bytes_moved = written > 0 || capture_scan.written_words > 0;
    let block_complete = evidence.result.is_ok()
        || error_status.is_some_and(|status| status.dma_status.transfer_done());
    let frame_sized = block_complete
        && written == expected
        && capture_scan.untouched_words == 0
        && guard_scan.written_words == 0;
    // `Ok` means the driver saw no bridge or host error when the block
    // completed.
    let bridge_errors = error_status.map_or(0, |status| {
        status.bridge_events.raw_status & BRIDGE_INT_ERROR_MASK
    });
    let host_error = error_status.is_some_and(|status| status.host_errors.any_error());
    let isp_errors = evidence.isp_int_raw & ISP_INT_ERROR_MASK;
    let valid = frame_sized && bridge_errors == 0 && !host_error && isp_errors == 0;

    Assessment {
        written,
        capture_scan,
        guard_scan,
        bytes_moved,
        block_complete,
        frame_sized,
        bridge_errors,
        host_error,
        isp_errors,
        valid,
    }
}

/// Prints the four result levels and returns whether the frame is valid.
fn print_verdict(label: &str, evidence: &CaptureEvidence<'_>) -> bool {
    let mode = evidence.mode;
    let expected = mode.expected_bytes();
    let Assessment {
        written,
        capture_scan,
        guard_scan,
        bytes_moved,
        block_complete,
        frame_sized,
        bridge_errors,
        host_error,
        isp_errors,
        valid,
    } = assess(evidence);

    println!("--- CSI CAPTURE VERDICT ({label}) ---");
    match evidence.result {
        Ok(frame) => println!(
            "{label}: receive=Ok received_bytes={}",
            frame.received_bytes
        ),
        Err(err) => println!("{label}: receive=Err({err})"),
    }
    println!("{label}: level1 bytes_moved={bytes_moved} written={written}");
    println!("{label}: level2 dma_block_complete={block_complete}");
    println!(
        "{label}: level3 frame_sized={} written={} expected={} sentinel_written_end={} untouched_words={} first_untouched={:?} guard_words_written={}",
        frame_sized,
        written,
        expected,
        capture_scan.written_end_byte,
        capture_scan.untouched_words,
        capture_scan.first_untouched_byte,
        guard_scan.written_words
    );
    print!("{label}: level4 valid_aligned_frame={valid} bridge_errors=0x{bridge_errors:02x}");
    print_bit_names(bridge_errors, BRIDGE_INT_NAMES);
    println!(" host_error={host_error}");
    print!("{label}: level4 isp_errors=0x{isp_errors:08x}");
    print_bit_names(isp_errors, ISP_INT_NAMES);
    println!();

    if mode == CaptureMode::Rgb565Psram && block_complete {
        if written == RGB565_FRAME_BYTES {
            println!(
                "{label}: BLOCK_TS was sized for RAW8 ({SC2336_FRAME_BYTES} bytes), the bridge ended the block at the RGB565 frame size"
            );
        } else if written == SC2336_FRAME_BYTES {
            println!(
                "{label}: the block ended at BLOCK_TS ({SC2336_FRAME_BYTES} bytes), half of the RGB565 frame"
            );
        } else {
            println!("{label}: the block ended after {written} bytes");
        }
    }

    if bytes_moved {
        let frame = &evidence.allocation[..written.min(expected)];
        print_frame_stats(label, frame);
        if evidence.test_pattern && frame_sized {
            print_test_pattern_rows(label, frame, mode.bytes_per_pixel());
        }
    }

    valid
}

fn print_host_detail_status(label: &str) {
    let host = esp_hal::peripherals::MIPI_CSI_HOST::regs();

    println!(
        "host-detail({}): msk phy_fatal=0x{:08x} pkt_fatal=0x{:08x} bndry=0x{:08x} seq=0x{:08x} crc=0x{:08x} pld_crc=0x{:08x} data_id=0x{:08x} ecc=0x{:08x} phy=0x{:08x}",
        label,
        host.int_msk_phy_fatal().read().bits(),
        host.int_msk_pkt_fatal().read().bits(),
        host.int_msk_bndry_frame_fatal().read().bits(),
        host.int_msk_seq_frame_fatal().read().bits(),
        host.int_msk_crc_frame_fatal().read().bits(),
        host.int_msk_pld_crc_fatal().read().bits(),
        host.int_msk_data_id().read().bits(),
        host.int_msk_ecc_corrected().read().bits(),
        host.int_msk_phy().read().bits(),
    );
    // Status registers are clear-on-read in IDF LL docs. We print them once at
    // end-of-capture to detect silent packet/PHY decode issues.
    println!(
        "host-detail({}): st phy_fatal=0x{:08x} pkt_fatal=0x{:08x} bndry=0x{:08x} seq=0x{:08x} crc=0x{:08x} pld_crc=0x{:08x} data_id=0x{:08x} ecc=0x{:08x} phy=0x{:08x}",
        label,
        host.int_st_phy_fatal().read().bits(),
        host.int_st_pkt_fatal().read().bits(),
        host.int_st_bndry_frame_fatal().read().bits(),
        host.int_st_seq_frame_fatal().read().bits(),
        host.int_st_crc_frame_fatal().read().bits(),
        host.int_st_pld_crc_fatal().read().bits(),
        host.int_st_data_id().read().bits(),
        host.int_st_ecc_corrected().read().bits(),
        host.int_st_phy().read().bits(),
    );
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();
    let delay = Delay::new();

    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    let mode = CaptureMode::selected();
    let test_pattern = env_flag(option_env!("CSI_TEST_PATTERN"));
    let host_errors = env_flag(option_env!("CSI_HOST_ERRORS"));
    let frame_export = env_flag(option_env!("CSI_FRAME_EXPORT"));
    let (dma_profile_name, dma_profile) = selected_dma_profile();
    let completion = Completion::selected();
    let exposure_lines = env_number("CSI_EXPOSURE_LINES", option_env!("CSI_EXPOSURE_LINES"));
    let gain_index = env_number("CSI_GAIN_INDEX", option_env!("CSI_GAIN_INDEX"));
    let repeat_frames = env_number("CSI_REPEAT_FRAMES", option_env!("CSI_REPEAT_FRAMES"));
    let coexist = env_flag(option_env!("CSI_DSI_COEXIST"));
    let continuous_sensor = env_flag(option_env!("CSI_SENSOR_CONTINUOUS"));
    let stream = env_flag(option_env!("CSI_STREAM"));
    let quiet = env_flag(option_env!("CSI_QUIET"));
    assert!(
        !coexist || repeat_frames.is_some(),
        "CSI_DSI_COEXIST needs CSI_REPEAT_FRAMES"
    );
    assert!(
        !stream || repeat_frames.is_some(),
        "CSI_STREAM needs CSI_REPEAT_FRAMES"
    );
    assert!(
        !continuous_sensor || repeat_frames.is_some(),
        "CSI_SENSOR_CONTINUOUS needs CSI_REPEAT_FRAMES"
    );
    if let Some(frames) = repeat_frames {
        assert!(
            (1..=MAX_REPEAT_FRAMES).contains(&frames),
            "CSI_REPEAT_FRAMES must be 1..={MAX_REPEAT_FRAMES}, got {frames}"
        );
        assert!(
            mode == CaptureMode::Raw8Psram,
            "CSI_REPEAT_FRAMES needs CSI_CAPTURE_MODE=A"
        );
    }
    println!(
        "SC2336 capture start: mode={} test_pattern={} host_errors_unmasked={} dma_profile={} frame_export={} exposure_lines={:?} gain_index={:?} repeat_frames={:?} completion={}",
        mode.label(),
        test_pattern,
        host_errors,
        dma_profile_name,
        frame_export,
        exposure_lines,
        gain_index,
        repeat_frames,
        completion.label()
    );
    println!("Board assumptions: SCCB SCL=GPIO8, SDA=GPIO7, addr=0x30");
    println!("Board assumptions: reset/pwdn/xclk are hardware-managed on this sub-board");

    if env_flag(option_env!("CSI_DPHY_LIFECYCLE")) {
        let mut i2c = I2c::new(
            peripherals.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(peripherals.GPIO7)
        .with_scl(peripherals.GPIO8);
        assert_eq!(
            verify_sc2336_id(&mut i2c).ok(),
            Some(SC2336_EXPECTED_PID),
            "SC2336 ID check failed"
        );
        program_sc2336_mode_1024x600_raw8(&mut i2c, &delay).expect("SC2336 mode programming");
        let ok = run_dphy_lifecycle(
            peripherals.MIPI_CSI,
            peripherals.VDMA_CH1,
            peripherals.MIPI_DSI,
            peripherals.VDMA_CH0,
            &mut i2c,
        );
        println!(
            "DPHY LIFECYCLE RESULT: {}",
            if ok { "PASS" } else { "FAIL" }
        );
        qa_result(
            ok,
            "dphy_lifecycle",
            format_args!("repeats={DPHY_LIFECYCLE_REPEATS}"),
        );
        loop {}
    }

    let mut lcd = if coexist {
        println!("CSI/DSI coexistence test");
        let mut lcd = Lcd::start(
            peripherals.MIPI_DSI,
            peripherals.VDMA_CH0,
            peripherals.GPIO27,
            peripherals.GPIO26,
            &delay,
        );
        println!("LCD streaming: started, dsi_channel=VDMA_CH0 csi_channel=VDMA_CH1");
        print_vdma_channel_state("after-dsi-start");
        if !lcd.baseline() {
            panic!("LCD-only baseline made no DSI progress");
        }
        Some(lcd)
    } else {
        None
    };

    if coexist {
        // `MipiDsi::new` already powered the shared rail, and only its drop
        // turns it off again.
        delay.delay_millis(20);
        println!("MIPI D-PHY 2.5V rail enabled by MIPI-DSI");
    } else {
        enable_mipi_dphy_power(&delay);
        delay.delay_millis(20);
        println!("MIPI D-PHY 2.5V rail enabled");
    }

    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO8);

    match verify_sc2336_id(&mut i2c) {
        Ok(pid) if pid == SC2336_EXPECTED_PID => println!("SC2336 ID RESULT: PASS"),
        Ok(pid) => {
            println!("SC2336 ID RESULT: FAIL (0x{:04x})", pid);
            panic!("SC2336 ID mismatch");
        }
        Err(err) => {
            println!("[FAIL] SC2336 ID read error: {:?}", err);
            panic!("SC2336 ID read failed");
        }
    }

    if let Err(err) = program_sc2336_mode_1024x600_raw8(&mut i2c, &delay) {
        println!("[FAIL] SC2336 mode programming failed: {:?}", err);
        panic!("SC2336 mode programming failed");
    }

    if test_pattern {
        match set_sc2336_test_pattern(&mut i2c, true) {
            Ok(value) => println!(
                "SC2336 test pattern enabled: [0x{:04x}]=0x{:02x}",
                SC2336_REG_TEST_PATTERN, value
            ),
            Err(err) => {
                println!("[FAIL] SC2336 test pattern write failed: {:?}", err);
                panic!("SC2336 test pattern write failed");
            }
        }
    }

    if let Err(err) = apply_sc2336_exposure_gain(&mut i2c, exposure_lines, gain_index) {
        println!("[FAIL] SC2336 exposure/gain write failed: {:?}", err);
        panic!("SC2336 exposure/gain write failed");
    }

    if let Err(err) = print_sc2336_regs(
        &mut i2c,
        "after mode programming, pre-stream",
        SC2336_KEY_MODE_REGS,
    ) {
        println!("[FAIL] SC2336 register readback failed: {:?}", err);
        panic!("SC2336 mode readback failed");
    }
    if let Err(err) = print_sc2336_exposure_gain(&mut i2c) {
        println!("[FAIL] SC2336 exposure/gain readback failed: {:?}", err);
        panic!("SC2336 exposure/gain readback failed");
    }

    println!(
        "SC2336 mode: RAW8 {}x{} @ 30 fps, lanes=2, mipi_clk={} Hz",
        SC2336_FRAME_WIDTH, SC2336_FRAME_HEIGHT, SC2336_MIPI_CLK_HZ
    );
    println!(
        "CSI lane bitrate config={} Mbps (DDR equivalent={} Mbps, if mipi_clk interpreted as clock-lane frequency)",
        SC2336_LANE_BIT_RATE_MBPS, SC2336_DDR_EQUIVALENT_MBPS
    );
    println!(
        "RAW8 frame bytes={} (transfer_units={} @ 64-bit)",
        SC2336_FRAME_BYTES, SC2336_FRAME_TRANSFER_UNITS
    );

    let allocation = allocate_capture_buffer(mode);
    if let Some(lcd) = &lcd {
        let overlap = lcd.overlaps(allocation);
        println!(
            "buffers: csi=0x{:08x}+{} dsi_fb0=0x{:08x}+{} dsi_fb1=0x{:08x}+{} overlap={}",
            allocation.as_ptr() as u32,
            allocation.len(),
            lcd.framebuffers[0] as u32,
            LCD_FB_BYTES,
            lcd.framebuffers[1] as u32,
            LCD_FB_BYTES,
            overlap
        );
        assert!(!overlap, "CSI buffer overlaps a DSI frame buffer");
    }

    let mut capture = MipiCsi::new(
        peripherals.MIPI_CSI,
        Config::default()
            .with_num_data_lanes(SC2336_LANES)
            .with_lane_bit_rate_mbps(SC2336_LANE_BIT_RATE_MBPS),
    )
    .unwrap()
    .with_dma(
        peripherals.VDMA_CH1,
        CaptureConfig::default()
            .with_frame_width(SC2336_FRAME_WIDTH)
            .with_frame_height(SC2336_FRAME_HEIGHT)
            .with_pixel_format(CapturePixelFormat::Raw8)
            .with_timeout(Duration::from_millis(500))
            .with_dma_axi_config(dma_profile),
    )
    .unwrap();

    let applied = dma_profile;
    println!(
        "dma-profile: name={} src_burst_len={} dst_burst_len={} src_outstanding={} dst_outstanding={}",
        dma_profile_name,
        applied.src_burst_len,
        applied.dst_burst_len,
        applied.src_outstanding_requests,
        applied.dst_outstanding_requests
    );

    apply_idf_default_bridge_data_type_filter();
    println!(
        "Applied IDF-style bridge data_type filter: min=0x{:02x}, max=0x{:02x}",
        CSI_IDF_DEFAULT_DATA_TYPE_MIN, CSI_IDF_DEFAULT_DATA_TYPE_MAX
    );

    let bridge_before_isp = debug_snapshot();
    if env_flag(option_env!("CSI_SKIP_ISP")) {
        assert!(!mode.isp_enabled(), "CSI_SKIP_ISP needs an ISP bypass mode");
        println!("isp-config: skipped (CSI_SKIP_ISP=1)");
    } else {
        configure_isp_like_idf(mode.isp_enabled());
    }
    let bridge_after_isp = debug_snapshot();
    print_bridge_state_transition(&bridge_before_isp, &bridge_after_isp, "isp-init");
    if host_errors {
        unmask_csi_host_errors();
    }
    print_capture_plan(mode, &bridge_after_isp, allocation);

    let stream_faults = env_flag(option_env!("CSI_STREAM_FAULTS"));
    let use_async = env_flag(option_env!("CSI_ASYNC"));
    assert!(!use_async || stream, "CSI_ASYNC needs CSI_STREAM");
    let test = match repeat_frames {
        _ if coexist && use_async => "csi_dsi_coexist_async",
        _ if coexist => "csi_dsi_coexist",
        Some(_) if use_async => "stream_async",
        Some(_) if stream && stream_faults => "stream_faults",
        Some(_) if stream => "stream",
        Some(_) => "repeat",
        None => "single_frame",
    };
    let mut failed_rounds = 0;
    let (capture, passed) = match repeat_frames {
        Some(frames) if stream => {
            let rounds: u32 = option_env!("CSI_FAULT_ROUNDS").map_or(1, |r| r.parse().unwrap());
            let (mut capture, mut passed) = (capture, true);
            for round in 0..rounds {
                if rounds > 1 {
                    println!("fault-round[{round}]: start");
                }
                let (c, ok) = if use_async {
                    run_async_stream_check(capture, &mut i2c, frames, lcd.as_mut())
                } else {
                    run_stream_capture(capture, &mut i2c, frames, test_pattern, quiet, lcd.as_mut())
                };
                capture = c;
                let ok = if ok && stream_faults {
                    let (c, ok) = run_stream_fault_cases(capture, &mut i2c);
                    capture = c;
                    ok
                } else {
                    ok
                };
                failed_rounds += u32::from(!ok);
                passed &= ok;
                if rounds > 1 {
                    println!("fault-round[{round}]: {}", if ok { "PASS" } else { "FAIL" });
                }
            }
            if rounds > 1 {
                println!("fault-rounds: total={rounds} failed={failed_rounds}");
            }
            (capture, passed)
        }
        Some(frames) => {
            let passed = run_repeated_capture(
                &mut capture,
                &mut i2c,
                allocation,
                test_pattern,
                frames,
                frame_export,
                completion,
                lcd.as_mut(),
                continuous_sensor,
                quiet,
            );
            (capture, passed)
        }
        None => {
            let passed = run_capture(
                &mut capture,
                &mut i2c,
                mode,
                allocation,
                test_pattern,
                frame_export,
                completion,
            );
            (capture, passed)
        }
    };
    let summary = format!(
        "mode={} completion={} frames={} failed_rounds={failed_rounds}",
        mode.label(),
        completion.label(),
        repeat_frames.unwrap_or(1)
    );
    if !passed {
        qa_result(false, test, format_args!("{summary}"));
        panic!("{}: no valid aligned frame", mode.label());
    }
    println!("{}: PASS", mode.label());

    if let Some(lcd) = lcd {
        let ok = coexistence_teardown(capture, lcd);
        println!("COEXISTENCE RESULT: {}", if ok { "PASS" } else { "FAIL" });
        qa_result(ok, test, format_args!("{summary}"));
    } else {
        drop(capture);
        qa_result(true, test, format_args!("{summary}"));
    }

    loop {}
}

/// The last line of a run, printed only after every check.
fn qa_result(pass: bool, test: &str, detail: core::fmt::Arguments<'_>) {
    println!(
        "QA_RESULT: {} test={test} {detail}",
        if pass { "PASS" } else { "FAIL" }
    );
}

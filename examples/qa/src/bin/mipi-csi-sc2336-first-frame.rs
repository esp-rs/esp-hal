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
//! - `CSI_CAPTURE_MODE=C`: ISP bypass, one 64 KiB DMA block into internal SRAM.
//! - `CSI_CAPTURE_MODE=D`: ISP bypass, one 64 KiB DMA block into PSRAM.
//! - `CSI_TEST_PATTERN=1`: SC2336 test pattern instead of the image.
//! - `CSI_HOST_ERRORS=1`: unmask the CSI host error interrupts, so that host errors end the
//!   capture.
//% CHIP_FILTER: mipi_csi_driver_supported && i2c_master_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

extern crate alloc;

use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    delay::Delay,
    dma::aligned::{DmaAlignedMut, InternalMemory},
    i2c::{
        self,
        master::{Config as I2cConfig, I2c},
    },
    main,
    mipi_csi::{
        CaptureConfig,
        CaptureDebugSnapshot,
        CaptureFrame,
        CapturePixelFormat,
        CaptureStatus,
        CaptureTrace,
        CaptureTraceEvent,
        Config,
        CsiCapture,
        DataLanes,
        DiscardContinueDiagnostic,
        MipiCsi,
        ReceiveError,
    },
    peripherals::{MIPI_CSI_BRIDGE, MIPI_CSI_HOST, PMU},
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
const DMA_INT_NAMES: &[(u32, &str)] = &[
    (0, "block_tfr_done"),
    (1, "dma_tfr_done"),
    (3, "src_transcomp"),
    (4, "dst_transcomp"),
    (28, "src_suspended"),
    (29, "suspended"),
    (30, "disabled"),
    (31, "aborted"),
];
// Bits 5 to 14 and 16 to 21 of `CH_INTSTATUS0`.
const DMA_INT_ERROR_MASK: u32 = 0x003F_7FE0;
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
    /// ISP bypass, one 64 KiB block into internal SRAM.
    Raw8InternalBlock,
    /// ISP bypass, one 64 KiB block into PSRAM.
    Raw8PsramBlock,
}

impl CaptureMode {
    fn selected() -> Self {
        match option_env!("CSI_CAPTURE_MODE") {
            Some("B") | Some("b") => Self::Rgb565Psram,
            Some("C") | Some("c") => Self::Raw8InternalBlock,
            Some("D") | Some("d") => Self::Raw8PsramBlock,
            _ => Self::Raw8Psram,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Raw8Psram => "A-raw8-bypass-psram",
            Self::Rgb565Psram => "B-rgb565-isp-psram",
            Self::Raw8InternalBlock => "C-raw8-bypass-sram-64k",
            Self::Raw8PsramBlock => "D-raw8-bypass-psram-64k",
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
            Self::Raw8InternalBlock => INTERNAL_CAPTURE_BYTES,
            Self::Raw8PsramBlock => INTERNAL_CAPTURE_BYTES,
        }
    }

    /// `BLOCK_TS` of the capture, in bytes.
    const fn dma_block_bytes(self) -> usize {
        match self {
            Self::Raw8InternalBlock | Self::Raw8PsramBlock => INTERNAL_CAPTURE_BYTES,
            _ => SC2336_FRAME_BYTES,
        }
    }

    const fn uses_internal_sram(self) -> bool {
        matches!(self, Self::Raw8InternalBlock)
    }

    const fn is_64k_block_mode(self) -> bool {
        matches!(self, Self::Raw8InternalBlock | Self::Raw8PsramBlock)
    }
}

fn env_flag(value: Option<&str>) -> bool {
    value == Some("1")
}

// Words after the capture slice. The driver panics when the DMA destination
// address passes the slice, so a changed guard word means that `DAR`
// under-reported the writes.
const GUARD_BYTES: usize = 4096;
const PSRAM_DMA_ALIGNMENT: usize = 128;
const SENTINEL_SEED: u32 = 0xA5C3_5A3C;
// Mode C sets both `BLOCK_TS` and the bridge block size (TRM §38.5.7) to the
// buffer size, so either one ends the block at the end of the buffer.
const INTERNAL_CAPTURE_BYTES: usize = 64 * 1024;
const INTERNAL_CAPTURE_BRIDGE_BURSTS: u16 =
    (INTERNAL_CAPTURE_BYTES / BRIDGE_DMA_BURST_BYTES) as u16;

static mut INTERNAL_CAPTURE: InternalMemory<[u8; INTERNAL_CAPTURE_BYTES + GUARD_BYTES]> =
    InternalMemory::new([0; INTERNAL_CAPTURE_BYTES + GUARD_BYTES]);

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

fn print_runtime_samples(samples: &[Option<esp_hal::mipi_csi::CaptureStatus>]) {
    println!("--- CSI DURING-CAPTURE SAMPLES ---");
    for (idx, sample) in samples.iter().enumerate() {
        match sample {
            Some(status) => {
                println!(
                    "sample[{}]: host=0x{:08x} bridge=0x{:08x} dma=0x{:08x} enabled={} done={} dma_units={}",
                    idx,
                    status.host_errors.raw_main_status,
                    status.bridge_events.raw_status,
                    status.dma_status.raw_status,
                    status.dma_status.channel_enabled,
                    status.dma_status.transfer_done(),
                    status.dma_status.completed_transfer_units
                );
            }
            None => println!("sample[{}]: <none>", idx),
        }
    }
}

fn print_runtime_phy_activity(summary: esp_hal::mipi_csi::CapturePhyActivitySummary) {
    println!("--- CSI ACTIVE-POLL PHY SUMMARY ---");
    println!(
        "samples: total={} active(bridge&&dma)={} bridge_enabled={} dma_enabled={}",
        summary.sample_count,
        summary.active_sample_count,
        summary.bridge_enabled_sample_count,
        summary.dma_enabled_sample_count
    );
    println!(
        "phy_stopstate: OR=0x{:08x} AND=0x{:08x}",
        summary.phy_stopstate_or_raw, summary.phy_stopstate_and_raw
    );
    println!(
        "phy_rx:        OR=0x{:08x} AND=0x{:08x}",
        summary.phy_rx_or_raw, summary.phy_rx_and_raw
    );
    println!(
        "decoded(active-only): clkactivehs_seen={} lane0_left_stopstate_seen={} lane1_left_stopstate_seen={}",
        summary.clkactivehs_seen,
        summary.data_lane0_left_stopstate_seen,
        summary.data_lane1_left_stopstate_seen
    );
    println!(
        "bridge(active-only): buf_depth_nonzero_seen={} buf_depth_max={}",
        summary.bridge_buf_depth_nonzero_seen, summary.bridge_buf_depth_max
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
    if mode.uses_internal_sram() {
        // SAFETY: `main` runs once, so this is the only reference to the static.
        let internal = unsafe { &mut *(&raw mut INTERNAL_CAPTURE) };
        return internal.get_mut().unsize().into_inner();
    }

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
) -> bool {
    let label = mode.label();
    let capture_len = mode.expected_bytes();

    let _ = print_isp_input_gate_readback("before-arm");
    print_capture_debug_snapshot("before-arm", &capture.debug_snapshot());
    clear_isp_interrupts();

    fill_sentinel(allocation);
    DmaAlignedMut::new(&mut *allocation)
        .expect("capture allocation is DMA aligned")
        .writeback();

    let mut stream_on = None;
    let result = capture.receive_with(&mut allocation[..capture_len], || {
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
    let after = capture.debug_snapshot();
    print_capture_debug_snapshot("after-shutdown", &after);
    print_dma_block_registers(label, &after);
    print!("{label}: isp int_raw at receive return=0x{isp_int_raw:08x}");
    print_bit_names(isp_int_raw, ISP_INT_NAMES);
    println!();
    let _ = print_isp_input_gate_readback("after-capture");
    print_host_detail_status("after-capture");
    print_runtime_samples(capture.runtime_status_samples());
    print_runtime_phy_activity(capture.runtime_phy_activity_summary());
    let trace = *capture.capture_trace();
    print_capture_trace(label, &trace);
    let discard = capture.discard_continue_diagnostic();
    print_discard_diagnostic(label, discard);
    if let Err(err) = &result {
        print_receive_error(label, err);
    }

    print_verdict(
        label,
        &CaptureEvidence {
            mode,
            result: &result,
            trace: &trace,
            discard,
            isp_int_raw,
            allocation,
            test_pattern,
        },
    )
}

fn print_capture_trace(label: &str, trace: &CaptureTrace) {
    println!(
        "{label}: trace polls={} dropped={} dar_start=0x{:08x}",
        trace.polls, trace.dropped, trace.dar_start
    );
    for entry in trace.entries() {
        print!(
            "{label}: trace t={}us {:?} written={} fifo={} ch_en={} abort_pending={} stopstate=0x{:08x} bridge=0x{:02x}",
            entry.elapsed_us,
            entry.event,
            entry.dar.wrapping_sub(trace.dar_start),
            entry.bridge_fifo_depth,
            entry.dma_enabled,
            entry.dma_abort_pending,
            entry.phy_stopstate,
            entry.bridge_int_raw
        );
        print_bit_names(entry.bridge_int_raw, BRIDGE_INT_NAMES);
        print!(" dma=0x{:08x}", entry.dma_intstatus);
        print_bit_names(entry.dma_intstatus & !DMA_INT_ERROR_MASK, DMA_INT_NAMES);
        println!(
            " dma_errors=0x{:08x}",
            entry.dma_intstatus & DMA_INT_ERROR_MASK
        );
    }
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

const DMA_INT_TRANSFER_DONE_BIT: u32 = 1 << 1;
const BRIDGE_INT_ERROR_MASK: u32 = 0x1F;

struct CaptureEvidence<'a> {
    mode: CaptureMode,
    result: &'a Result<CaptureFrame, ReceiveError>,
    trace: &'a CaptureTrace,
    discard: DiscardContinueDiagnostic,
    isp_int_raw: u32,
    allocation: &'a [u8],
    test_pattern: bool,
}

/// Prints the four result levels and returns whether the frame is valid.
fn print_verdict(label: &str, evidence: &CaptureEvidence<'_>) -> bool {
    let mode = evidence.mode;
    let trace = evidence.trace;
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

    let entries = trace.entries();
    let written = entries
        .last()
        .map_or(0, |entry| entry.dar.wrapping_sub(trace.dar_start) as usize);
    let first_data = entries
        .iter()
        .find(|entry| entry.event == CaptureTraceEvent::FirstData);
    let data_before_stream_on = entries
        .iter()
        .find(|entry| entry.event == CaptureTraceEvent::SourceStarted)
        .is_some_and(|entry| entry.dar != trace.dar_start);
    let completion = entries.iter().find(|entry| {
        entry.event == CaptureTraceEvent::DmaEvent
            && entry.dma_intstatus & DMA_INT_TRANSFER_DONE_BIT != 0
    });
    let stalls = entries
        .iter()
        .filter(|entry| entry.event == CaptureTraceEvent::Stalled)
        .count();

    let capture_scan = scan_sentinel(evidence.allocation, 0..expected);
    let guard_scan = scan_sentinel(evidence.allocation, expected..evidence.allocation.len());

    let bytes_moved = written > 0 || capture_scan.written_words > 0;
    let block_complete = evidence.result.is_ok()
        || error_status.is_some_and(|status| status.dma_status.transfer_done());
    let frame_sized = block_complete
        && written == expected
        && capture_scan.untouched_words == 0
        && guard_scan.written_words == 0;
    // Errors after the block completed belong to the next frame, which no DMA
    // block was waiting for.
    let wait_end = entries
        .iter()
        .find(|entry| entry.event == CaptureTraceEvent::WaitEnded);
    let bridge_errors_at = if completion.is_some() {
        "completion"
    } else {
        "wait_end"
    };
    let bridge_errors = completion
        .or(wait_end)
        .map_or(0, |entry| entry.bridge_int_raw & BRIDGE_INT_ERROR_MASK);
    let host_error = error_status.is_some_and(|status| status.host_errors.any_error());
    let isp_errors = evidence.isp_int_raw & ISP_INT_ERROR_MASK;
    // Mode C stops the DMA mid-frame, so the ISP overflows behind it.
    let isp_errors_count = !mode.is_64k_block_mode();
    let valid = frame_sized
        && bridge_errors == 0
        && evidence.discard.discard_count == 0
        && !host_error
        && (isp_errors == 0 || !isp_errors_count)
        && !data_before_stream_on;

    println!("--- CSI CAPTURE VERDICT ({label}) ---");
    match evidence.result {
        Ok(frame) => println!(
            "{label}: receive=Ok received_bytes={}",
            frame.received_bytes
        ),
        Err(err) => println!("{label}: receive=Err({err})"),
    }
    println!(
        "{label}: level1 bytes_moved={} written={} first_data_at={:?}us stalls={}",
        bytes_moved,
        written,
        first_data.map(|entry| entry.elapsed_us),
        stalls
    );
    println!(
        "{label}: level2 dma_block_complete={} completed_at={:?}us",
        block_complete,
        completion.map(|entry| entry.elapsed_us)
    );
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
    print!(
        "{label}: level4 valid_aligned_frame={} bridge_errors(at {})=0x{:02x}",
        valid, bridge_errors_at, bridge_errors
    );
    print_bit_names(bridge_errors, BRIDGE_INT_NAMES);
    println!(
        " discard_count={} host_error={} data_before_stream_on={}",
        evidence.discard.discard_count, host_error, data_before_stream_on
    );
    print!("{label}: level4 isp_errors=0x{isp_errors:08x}");
    print_bit_names(isp_errors, ISP_INT_NAMES);
    if !isp_errors_count {
        print!(" (not counted)");
    }
    println!();

    match mode {
        CaptureMode::Rgb565Psram if block_complete => {
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
        CaptureMode::Raw8InternalBlock | CaptureMode::Raw8PsramBlock => println!(
            "{label}: this mode stops after {INTERNAL_CAPTURE_BYTES} bytes by design, it is not a full frame"
        ),
        _ => {}
    }

    let shutdown = entries
        .iter()
        .rev()
        .find_map(|entry| match entry.event {
            CaptureTraceEvent::ChannelDisabled => Some("CH_EN cleared after disable"),
            CaptureTraceEvent::ChannelAborted => Some("stopped only after suspend and abort"),
            CaptureTraceEvent::ShutdownFailed => Some("not shown stopped"),
            _ => None,
        })
        .unwrap_or("already stopped when the wait ended");
    if let Some(last) = entries.last() {
        println!(
            "{label}: shutdown={} last_event={:?} ch_en={} abort_pending={} bridge_fifo={}",
            shutdown, last.event, last.dma_enabled, last.dma_abort_pending, last.bridge_fifo_depth
        );
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

fn print_discard_diagnostic(label: &str, report: esp_hal::mipi_csi::DiscardContinueDiagnostic) {
    if !report.enabled {
        return;
    }

    let dar_delta = report.dar_end.wrapping_sub(report.dar_start);
    println!(
        "{label}: discard-diag count={} fatal_error={} dma_completed={} valid_complete_frame={} units={}/{} dar_start=0x{:08x} dar_end=0x{:08x} dar_delta={}",
        report.discard_count,
        report.fatal_error_seen,
        report.dma_completed,
        report.valid_complete_frame,
        report.completed_transfer_units,
        report.expected_transfer_units,
        report.dar_start,
        report.dar_end,
        dar_delta
    );
    print!("{label}: discard-diag dar_samples =");
    for sample in report.dar_samples {
        match sample {
            Some(dar) => print!(" 0x{dar:08x}"),
            None => break,
        }
    }
    println!();
    if report.discard_count > 0 {
        println!(
            "{label}: DISCARD is not a valid frame boundary. No DMA re-arm was performed, so a completed block can still be a spliced partial frame."
        );
    }
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
    println!(
        "SC2336 capture start: mode={} test_pattern={} host_errors_unmasked={}",
        mode.label(),
        test_pattern,
        host_errors
    );
    println!("Board assumptions: SCCB SCL=GPIO8, SDA=GPIO7, addr=0x30");
    println!("Board assumptions: reset/pwdn/xclk are hardware-managed on this sub-board");

    enable_mipi_dphy_power(&delay);
    delay.delay_millis(20);
    println!("MIPI D-PHY 2.5V rail enabled");

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

    if let Err(err) = print_sc2336_regs(
        &mut i2c,
        "after mode programming, pre-stream",
        SC2336_KEY_MODE_REGS,
    ) {
        println!("[FAIL] SC2336 register readback failed: {:?}", err);
        panic!("SC2336 mode readback failed");
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
            .with_timeout(Duration::from_millis(500)),
    )
    .unwrap();

    apply_idf_default_bridge_data_type_filter();
    println!(
        "Applied IDF-style bridge data_type filter: min=0x{:02x}, max=0x{:02x}",
        CSI_IDF_DEFAULT_DATA_TYPE_MIN, CSI_IDF_DEFAULT_DATA_TYPE_MAX
    );

    if mode.is_64k_block_mode() {
        capture
            .set_diagnostic_dma_transfer_bytes(INTERNAL_CAPTURE_BYTES)
            .expect("64 KiB is 8-byte aligned and fits one DMA block");
        MIPI_CSI_BRIDGE::regs()
            .dmablk_size()
            .write(|w| unsafe { w.dmablk_size().bits(INTERNAL_CAPTURE_BRIDGE_BURSTS) });
    }
    // A lone DISCARD does not end the wait, so the trace shows what follows it.
    capture.enable_discard_continue_diagnostic();

    let bridge_before_isp = capture.debug_snapshot();
    configure_isp_like_idf(mode.isp_enabled());
    let bridge_after_isp = capture.debug_snapshot();
    print_bridge_state_transition(&bridge_before_isp, &bridge_after_isp, "isp-init");
    if host_errors {
        unmask_csi_host_errors();
    }
    print_capture_plan(mode, &bridge_after_isp, allocation);

    if !run_capture(&mut capture, &mut i2c, mode, allocation, test_pattern) {
        panic!("{}: no valid aligned frame", mode.label());
    }
    println!("{}: PASS", mode.label());

    loop {}
}

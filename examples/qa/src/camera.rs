//! The SC2336 camera of the ESP32-P4 Function EV Board, and the frame checks
//! that the MIPI-CSI QA tests share.
//!
//! The camera sub-board handles the sensor reset, power-down and clock. The
//! sensor answers on SCCB at address 0x30, with SDA on GPIO7 and SCL on GPIO8.

use core::{
    future::Future,
    pin::Pin,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};

use esp_hal::{
    Blocking,
    delay::Delay,
    dma::aligned::DmaAlignedMut,
    i2c::master::{Config as I2cConfig, Error, I2c},
    mipi_csi::{CaptureConfig, CapturePixelFormat, Config, DataLanes},
    peripherals::{GPIO7, GPIO8, I2C0},
    time::{Duration, Instant, Rate},
};

pub const WIDTH: u16 = 1024;
pub const HEIGHT: u16 = 600;
/// Bytes of one RAW8 frame.
pub const FRAME_BYTES: usize = WIDTH as usize * HEIGHT as usize;
/// Bytes after each frame buffer that the DMA must not write.
pub const GUARD_BYTES: usize = 4096;
/// Alignment of the frame buffers in PSRAM.
pub const PSRAM_DMA_ALIGNMENT: usize = 128;

const ADDRESS: u8 = 0x30;
const EXPECTED_ID: u16 = 0xcb3a;
const REG_ID_H: u16 = 0x3107;
const REG_ID_L: u16 = 0x3108;
const REG_STREAM: u16 = 0x0100;
// `sc2336_set_test_pattern` in esp-cam-sensor `sc2336.c`.
const REG_TEST_PATTERN: u16 = 0x4501;
const TEST_PATTERN_BIT: u8 = 1 << 3;
const REG_EXPOSURE_H: u16 = 0x3e00;
const REG_EXPOSURE_M: u16 = 0x3e01;
const REG_EXPOSURE_L: u16 = 0x3e02;
const REG_DIG_COARSE_GAIN: u16 = 0x3e06;
const REG_DIG_FINE_GAIN: u16 = 0x3e07;
const REG_ANALOG_GAIN: u16 = 0x3e09;
/// 10 ms of the 33 ms frame, the exposure of the `mipi_csi_camera_lcd` example.
const EXPOSURE_LINES: u32 = 300;

// Espressif reference register list:
// sc2336_mipi_2lane_24Minput_1024x600_raw8_30fps.
const MODE_1024X600_RAW8_30FPS: &[(u16, u8)] = &[
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

/// The CSI host settings of the sensor mode: two lanes at the mode's MIPI clock
/// of 288 MHz.
pub fn csi_config() -> Config {
    Config::default()
        .with_num_data_lanes(DataLanes::_2)
        .with_lane_bit_rate_mbps(288)
}

/// One RAW8 frame of the sensor mode, with a timeout of about 15 frames.
pub fn capture_config() -> CaptureConfig {
    CaptureConfig::default()
        .with_frame_width(WIDTH)
        .with_frame_height(HEIGHT)
        .with_pixel_format(CapturePixelFormat::Raw8)
        .with_timeout(Duration::from_millis(500))
}

pub struct Sc2336<'d> {
    i2c: I2c<'d, Blocking>,
}

impl<'d> Sc2336<'d> {
    /// Checks the sensor ID, then programs the 1024x600 RAW8 30 fps mode with
    /// the stream off, an exposure of 10 ms and unit gain.
    pub fn new(i2c: I2C0<'d>, sda: GPIO7<'d>, scl: GPIO8<'d>, test_pattern: bool) -> Self {
        let i2c = I2c::new(
            i2c,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);
        let mut sensor = Self { i2c };
        let id = u16::from_be_bytes([
            sensor
                .read(REG_ID_H)
                .expect("SC2336 does not answer on SCCB"),
            sensor
                .read(REG_ID_L)
                .expect("SC2336 does not answer on SCCB"),
        ]);
        assert_eq!(id, EXPECTED_ID, "unexpected SC2336 ID {id:#06x}");

        for &(reg, value) in MODE_1024X600_RAW8_30FPS {
            sensor.write(reg, value).expect("SC2336 mode setup failed");
        }
        Delay::new().delay_millis(5);

        // The low 4 bits of 0x3e02 are a fraction of a line and stay 0.
        let exposure_gain = [
            (REG_EXPOSURE_H, ((EXPOSURE_LINES >> 12) & 0xf) as u8),
            (REG_EXPOSURE_M, ((EXPOSURE_LINES >> 4) & 0xff) as u8),
            (REG_EXPOSURE_L, ((EXPOSURE_LINES & 0xf) << 4) as u8),
            (REG_DIG_FINE_GAIN, 0x80),
            (REG_DIG_COARSE_GAIN, 0x00),
            (REG_ANALOG_GAIN, 0x00),
        ];
        for (reg, value) in exposure_gain {
            sensor
                .write(reg, value)
                .expect("SC2336 exposure setup failed");
        }

        if test_pattern {
            let value = sensor.read(REG_TEST_PATTERN).unwrap();
            sensor
                .write(REG_TEST_PATTERN, value | TEST_PATTERN_BIT)
                .expect("SC2336 test pattern setup failed");
        }
        sensor
    }

    fn read(&mut self, reg: u16) -> Result<u8, Error> {
        let mut value = [0];
        self.i2c
            .write_read(ADDRESS, &reg.to_be_bytes(), &mut value)?;
        Ok(value[0])
    }

    fn write(&mut self, reg: u16, value: u8) -> Result<(), Error> {
        let [high, low] = reg.to_be_bytes();
        self.i2c.write(ADDRESS, &[high, low, value])
    }

    pub fn set_stream(&mut self, on: bool) -> Result<(), Error> {
        self.write(REG_STREAM, on as u8)
    }
}

const SENTINEL_SEED: u32 = 0xA5C3_5A3C;

fn sentinel_word(index: usize) -> u32 {
    (index as u32).wrapping_mul(0x9E37_79B1) ^ SENTINEL_SEED
}

/// Fills `region` with a sentinel pattern and writes it back to memory, so
/// that [`sentinel_words`] later shows which words the DMA did not write.
pub fn fill_sentinel(region: &mut [u8]) {
    for (index, word) in region.as_chunks_mut::<4>().0.iter_mut().enumerate() {
        *word = sentinel_word(index).to_le_bytes();
    }
    DmaAlignedMut::new(region)
        .expect("region is DMA aligned")
        .writeback();
}

/// Invalidates `region` and returns how many of its words still hold the
/// pattern of [`fill_sentinel`].
pub fn sentinel_words(region: &mut [u8]) -> usize {
    DmaAlignedMut::new(&mut *region)
        .expect("region is DMA aligned")
        .invalidate();
    count_sentinel(region)
}

/// [`sentinel_words`] for memory that the driver already invalidated, such as
/// a stream frame.
pub fn count_sentinel(region: &[u8]) -> usize {
    region
        .as_chunks::<4>()
        .0
        .iter()
        .enumerate()
        .filter(|&(index, word)| u32::from_le_bytes(*word) == sentinel_word(index))
        .count()
}

/// FNV-style checksum, printed so that runs can compare frames.
pub fn checksum32(data: &[u8]) -> u32 {
    data.iter()
        .fold(0u32, |acc, &byte| acc.wrapping_mul(16777619) ^ byte as u32)
}

static WOKEN: AtomicBool = AtomicBool::new(false);
static WOKEN_POLLS: AtomicU32 = AtomicU32::new(0);

fn flag_waker() -> Waker {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(core::ptr::null(), &VTABLE),
        |_| WOKEN.store(true, Ordering::Release),
        |_| WOKEN.store(true, Ordering::Release),
        |_| {},
    );
    // SAFETY: the vtable functions ignore the data pointer.
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
}

/// Polls `future` once, then again only after its waker fired, until it
/// completes or `timeout` passes. A future that never wakes times out.
pub fn poll_on_wake<F: Future>(mut future: Pin<&mut F>, timeout: Duration) -> Option<F::Output> {
    let waker = flag_waker();
    let mut cx = Context::from_waker(&waker);
    let deadline = Instant::now() + timeout;
    loop {
        WOKEN.store(false, Ordering::Release);
        if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return Some(output);
        }
        while !WOKEN.load(Ordering::Acquire) {
            if Instant::now() > deadline {
                return None;
            }
        }
        WOKEN_POLLS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Polls of [`poll_on_wake`] that returned `Pending` and were followed by a
/// wake.
pub fn woken_polls() -> u32 {
    WOKEN_POLLS.load(Ordering::Relaxed)
}

/// Polls `future` once and returns whether it is pending.
pub fn is_pending<F: Future>(future: Pin<&mut F>) -> bool {
    future
        .poll(&mut Context::from_waker(&flag_waker()))
        .is_pending()
}

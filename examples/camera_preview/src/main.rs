//! A live camera preview on the ESP32-P4-Function-EV-Board.
//!
//! The SC2336 camera sends 1024x600 RAW8 Bayer frames over two MIPI-CSI lanes
//! into PSRAM. The CPU converts the newest frame into the RGB565 frame buffer
//! of the EK79007 MIPI-DSI LCD, which has the same size. The first frame sets
//! the white balance.
//!
//! Board wiring (fixed):
//! - SC2336 SCCB => SDA GPIO7, SCL GPIO8
//! - LCD reset => GPIO27
//! - LCD backlight => GPIO26

//% CHIP_FILTER: mipi_csi_driver_supported && mipi_dsi_driver_supported
//% TAG: complex

#![no_std]
#![no_main]

extern crate alloc;

mod demosaic;
mod sc2336;

use core::alloc::Layout;

use demosaic::{Rgb565Lut, bayer_to_rgb565};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::{
        CpuClock,
        ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    },
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    main,
    mipi_csi::{
        CaptureConfig,
        CapturePixelFormat,
        Config as CsiConfig,
        DataLanes,
        MipiCsi,
        StreamStats,
    },
    mipi_dsi::{
        Config as DsiConfig,
        DataLanes as DsiDataLanes,
        MipiDsi,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, DsiDpi, FrameTiming},
    },
    peripherals::{MIPI_DSI, VDMA_CH0},
    psram,
    time::{Duration, Instant, Rate},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const WIDTH: usize = sc2336::WIDTH as usize;
const HEIGHT: usize = sc2336::HEIGHT as usize;
const RAW_BYTES: usize = WIDTH * HEIGHT;
const RGB565_BYTES: usize = WIDTH * HEIGHT * 2;
/// Cache-line alignment of PSRAM DMA buffers.
const DMA_ALIGNMENT: usize = 128;
const EXPOSURE_LINES: u32 = 300;
const FRAME_TIMEOUT: Duration = Duration::from_millis(500);
/// Two LCD frames take about 34 ms.
const COMMIT_TIMEOUT: Duration = Duration::from_millis(100);
const REPORT_INTERVAL: Duration = Duration::from_secs(5);

/// EK79007 vendor init sequence.
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

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );
    let delay = Delay::new();

    let _reset = {
        let mut reset = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());
        delay.delay_millis(10);
        reset.set_high();
        delay.delay_millis(120);
        reset
    };
    let _backlight = Output::new(peripherals.GPIO26, Level::High, OutputConfig::default());
    let mut dpi = start_lcd(peripherals.MIPI_DSI, peripherals.VDMA_CH0, &delay);
    println!("LCD: EK79007 {WIDTH}x{HEIGHT} RGB565");

    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO8);
    let id = sc2336::read_id(&mut i2c).expect("SC2336 does not answer on SCCB");
    assert_eq!(id, sc2336::EXPECTED_ID, "unexpected sensor id 0x{id:04x}");
    sc2336::configure(&mut i2c, &delay).expect("SC2336 mode setup failed");
    sc2336::set_exposure_unit_gain(&mut i2c, EXPOSURE_LINES).expect("SC2336 exposure setup failed");
    println!(
        "SC2336: {WIDTH}x{HEIGHT} RAW8 30 fps, {} lanes at {} Mbps, exposure {} us",
        sc2336::LANES,
        sc2336::LANE_BIT_RATE_MBPS,
        EXPOSURE_LINES * sc2336::LINE_NS / 1000,
    );

    let capture = MipiCsi::new(
        peripherals.MIPI_CSI,
        CsiConfig::default()
            .with_num_data_lanes(DataLanes::_2)
            .with_lane_bit_rate_mbps(sc2336::LANE_BIT_RATE_MBPS),
    )
    .unwrap()
    .with_dma(
        peripherals.VDMA_CH1,
        CaptureConfig::default()
            .with_frame_width(sc2336::WIDTH)
            .with_frame_height(sc2336::HEIGHT)
            .with_pixel_format(CapturePixelFormat::Raw8)
            .with_timeout(FRAME_TIMEOUT),
    )
    .unwrap();

    let mut stream = match capture.into_stream([alloc_dma(RAW_BYTES), alloc_dma(RAW_BYTES)]) {
        Ok(stream) => stream,
        Err((_, _, error)) => panic!("stream did not start: {error:?}"),
    };
    sc2336::set_stream(&mut i2c, true).expect("SC2336 stream on failed");

    let lut = {
        let frame = stream
            .next_frame(FRAME_TIMEOUT)
            .expect("no first frame from the camera");
        let means = demosaic::channel_means(&frame, WIDTH, HEIGHT, sc2336::BAYER_ORDER);
        let gains = demosaic::gray_world_gains(means);
        println!(
            "white balance: mean r={} g={} b={}, gains_q8 r={} g={} b={}",
            means[0], means[1], means[2], gains[0], gains[1], gains[2]
        );
        Rgb565Lut::new(gains)
    };

    let mut report = Report::new(stream.stats());
    loop {
        let frame = match stream.next_frame(FRAME_TIMEOUT) {
            Ok(frame) => frame,
            Err(error) => {
                println!("no frame: {error:?}");
                continue;
            }
        };
        let start = Instant::now();
        bayer_to_rgb565(
            &frame,
            WIDTH,
            HEIGHT,
            sc2336::BAYER_ORDER,
            &lut,
            dpi.framebuffer_mut(),
        );
        drop(frame);
        report.add(start.elapsed());

        dpi.commit();
        dpi.wait_for_commit(COMMIT_TIMEOUT)
            .expect("the LCD DMA stopped");

        if report.start.elapsed() >= REPORT_INTERVAL {
            report = report.print(stream.stats());
        }
    }
}

fn start_lcd(dsi: MIPI_DSI<'static>, channel: VDMA_CH0<'static>, delay: &Delay) -> DsiDpi<'static> {
    let mut bus = MipiDsi::new(
        dsi,
        channel,
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
        // Sleep out, then display on.
        dbi.write_cmd(0x11, &[]).unwrap();
        delay.delay_millis(120);
        dbi.write_cmd(0x29, &[]).unwrap();
        delay.delay_millis(20);
    }

    let framebuffers = [alloc_dma(RGB565_BYTES), alloc_dma(RGB565_BYTES)];
    bus.dpi(
        DpiConfig {
            virtual_channel: 0,
            pixel_clock_mhz: 48.0,
            dpi_clk_src: DpiClockSource::PllF240m,
            in_color_format: ColorFormat::Rgb565,
            out_color_format: ColorFormat::Rgb565,
            timing: FrameTiming {
                h_active: WIDTH as u32,
                hsw: 10,
                hbp: 120,
                hfp: 120,
                v_active: HEIGHT as u32,
                vsw: 1,
                vbp: 20,
                vfp: 20,
            },
        },
        &framebuffers,
    )
    .expect("DPI init failed")
}

/// Frame rates and conversion time over one report interval.
struct Report {
    start: Instant,
    stats: StreamStats,
    shown: u32,
    convert: Duration,
}

impl Report {
    fn new(stats: StreamStats) -> Self {
        Self {
            start: Instant::now(),
            stats,
            shown: 0,
            convert: Duration::ZERO,
        }
    }

    fn add(&mut self, convert: Duration) {
        self.shown += 1;
        self.convert += convert;
    }

    fn print(self, stats: StreamStats) -> Self {
        let elapsed_ms = self.start.elapsed().as_millis().max(1);
        let fps = |frames: u32| frames as u64 * 1000 / elapsed_ms;
        println!(
            "camera {} fps, shown {} fps, overwritten {}, errored {}, convert {} ms",
            fps(stats.completed - self.stats.completed),
            fps(self.shown),
            stats.overwritten - self.stats.overwritten,
            stats.errored - self.stats.errored,
            self.convert.as_millis() / self.shown.max(1) as u64,
        );
        Self::new(stats)
    }
}

/// A zeroed PSRAM buffer that DMA can use.
fn alloc_dma(len: usize) -> &'static mut [u8] {
    let layout = Layout::from_size_align(len, DMA_ALIGNMENT).unwrap();
    let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
    assert!(!ptr.is_null(), "PSRAM allocation failed");
    unsafe { core::slice::from_raw_parts_mut(ptr, len) }
}

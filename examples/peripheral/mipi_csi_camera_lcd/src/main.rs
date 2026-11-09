//! Shows the SC2336 camera on the EK79007 MIPI-DSI panel of the
//! ESP32-P4-Function-EV-Board.
//!
//! The camera sends 1024×600 RAW8 Bayer frames over two MIPI-CSI lanes into
//! PSRAM. The CPU demosaics each frame into the RGB565 frame buffer of the
//! panel, which has the same size.
//!
//! By default one frame is captured and stays on the screen. With the `live`
//! feature the example then captures continuously and always shows the newest
//! frame. The `color-bars` feature shows white, yellow, cyan, green, magenta,
//! red, blue and black bars instead, from left to right, to check the colour
//! channels of the panel. The `white-balance` feature scales red and blue so
//! that their mean matches green in the snapshot.
//!
//! Board wiring (fixed):
//! - SC2336 SCCB => SDA GPIO7, SCL GPIO8
//! - LCD reset   => GPIO27
//! - LCD backlight => GPIO26

//% CHIP_FILTER: mipi_csi_driver_supported && mipi_dsi_driver_supported

#![no_std]
#![no_main]

extern crate alloc;

mod demosaic;
mod sc2336;

use core::alloc::Layout;

use demosaic::{GAIN_ONE, Rgb565Lut, bayer_to_rgb565};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    clock::{
        CpuClock,
        ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    },
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    main,
    mipi_csi::{CaptureConfig, CapturePixelFormat, Config as CsiConfig, DataLanes, MipiCsi},
    mipi_dsi::{
        Config as DsiConfig,
        DataLanes as DsiDataLanes,
        MipiDsi,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, DsiDpi, FrameTiming},
    },
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
    let mut dpi = start_panel(peripherals.MIPI_DSI, peripherals.VDMA_CH0, &delay);
    println!("LCD: EK79007 {WIDTH}x{HEIGHT} RGB565, frame buffer {RGB565_BYTES} bytes");

    if cfg!(feature = "color-bars") {
        draw_color_bars(dpi.framebuffer_mut());
        dpi.commit();
        dpi.wait_for_commit();
        println!("color bars: white yellow cyan green magenta red blue black, left to right");
        loop {
            delay.delay_millis(1000);
        }
    }

    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO8);
    let id = sc2336::read_id(&mut i2c).expect("SC2336 does not answer on SCCB");
    println!(
        "SC2336: id=0x{id:04x} expected=0x{:04x}",
        sc2336::EXPECTED_ID
    );
    assert_eq!(id, sc2336::EXPECTED_ID, "unexpected sensor id");
    sc2336::configure(&mut i2c, &delay).expect("SC2336 mode setup failed");
    sc2336::set_exposure_unit_gain(&mut i2c, EXPOSURE_LINES).expect("SC2336 exposure setup failed");
    println!(
        "SC2336: {WIDTH}x{HEIGHT} RAW8 30 fps, {} lanes at {} Mbps, exposure {EXPOSURE_LINES} lines ({} us), gain 1x, bayer {:?}",
        sc2336::LANES,
        sc2336::LANE_BIT_RATE_MBPS,
        EXPOSURE_LINES * sc2336::LINE_NS / 1000,
        sc2336::BAYER_ORDER
    );

    let mut capture = MipiCsi::new(
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

    let raw = [alloc_dma(RAW_BYTES), alloc_dma(RAW_BYTES)];

    // Snapshot. The sensor starts streaming once the capture is armed, so the
    // first frame is whole.
    let mut stream_on = Ok(());
    let frame = capture.receive_with(raw[0], || stream_on = sc2336::set_stream(&mut i2c, true));
    let stream_off = sc2336::set_stream(&mut i2c, false);
    println!("snapshot: capture={frame:?} stream_on={stream_on:?} stream_off={stream_off:?}");
    let frame = frame.expect("snapshot capture failed");
    assert_eq!(
        frame.received_bytes, RAW_BYTES,
        "snapshot has the wrong size"
    );

    let (min, max) = raw[0]
        .iter()
        .fold((u8::MAX, 0), |(lo, hi), &v| (lo.min(v), hi.max(v)));
    let means = demosaic::channel_means(raw[0], WIDTH, HEIGHT, sc2336::BAYER_ORDER);
    println!(
        "snapshot: {} bytes checksum32=0x{:08x} min={min} max={max} mean r={} g={} b={}",
        frame.received_bytes,
        checksum32(raw[0]),
        means[0],
        means[1],
        means[2]
    );
    let gains = if cfg!(feature = "white-balance") {
        demosaic::gray_world_gains(means)
    } else {
        [GAIN_ONE; 3]
    };
    println!(
        "rgb565: gains_q8 r={} g={} b={}",
        gains[0], gains[1], gains[2]
    );
    let lut = Rgb565Lut::new(gains);

    let start = Instant::now();
    bayer_to_rgb565(
        raw[0],
        WIDTH,
        HEIGHT,
        sc2336::BAYER_ORDER,
        &lut,
        dpi.framebuffer_mut(),
    );
    let convert_us = start.elapsed().as_micros();
    let start = Instant::now();
    dpi.commit();
    dpi.wait_for_commit();
    println!(
        "snapshot: converted {RGB565_BYTES} bytes in {} ms, shown after {} us",
        convert_us / 1000,
        start.elapsed().as_micros()
    );

    if cfg!(feature = "live") {
        live(capture, raw, &mut i2c, &mut dpi, &lut);
    }
    loop {
        delay.delay_millis(1000);
    }
}

fn start_panel(
    dsi: esp_hal::peripherals::MIPI_DSI<'static>,
    channel: esp_hal::peripherals::VDMA_CH0<'static>,
    delay: &Delay,
) -> DsiDpi<'static> {
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

/// Shows the newest camera frame continuously.
fn live(
    capture: esp_hal::mipi_csi::CsiCapture<'static>,
    raw: [&'static mut [u8]; 2],
    i2c: &mut I2c<'_, Blocking>,
    dpi: &mut DsiDpi<'static>,
    lut: &Rgb565Lut,
) {
    let mut stream = match capture.into_stream(raw) {
        Ok(stream) => stream,
        Err((_, _, error)) => panic!("stream did not start: {error:?}"),
    };
    sc2336::set_stream(i2c, true).expect("SC2336 stream on failed");

    let mut window = Window::new(stream.stats(), dpi.dma_interrupt_count());
    let mut presented = 0u32;
    loop {
        let frame = match stream.next_frame(FRAME_TIMEOUT) {
            Ok(frame) => frame,
            Err(error) => {
                window.timeouts += 1;
                println!("live: no frame: {error:?}");
                continue;
            }
        };
        let sequence = frame.sequence();
        let start = Instant::now();
        bayer_to_rgb565(
            &frame,
            WIDTH,
            HEIGHT,
            sc2336::BAYER_ORDER,
            lut,
            dpi.framebuffer_mut(),
        );
        drop(frame);
        let convert_us = start.elapsed().as_micros();
        let start = Instant::now();
        dpi.commit();
        dpi.wait_for_commit();
        window.add(convert_us, start.elapsed().as_micros());
        presented += 1;

        if matches!(presented, 1 | 10 | 100) || presented % 1000 == 0 {
            println!(
                "live[{presented}]: sequence={sequence} stats={:?}",
                stream.stats()
            );
        }
        if window.start.elapsed() >= Duration::from_secs(5) {
            window = window.report(stream.stats(), dpi.dma_interrupt_count());
        }
    }
}

/// Rates over one reporting interval of the live preview.
struct Window {
    start: Instant,
    stats: esp_hal::mipi_csi::StreamStats,
    dsi_irqs: u32,
    presented: u32,
    timeouts: u32,
    convert_us: u64,
    max_convert_us: u64,
    commit_us: u64,
}

impl Window {
    fn new(stats: esp_hal::mipi_csi::StreamStats, dsi_irqs: u32) -> Self {
        Self {
            start: Instant::now(),
            stats,
            dsi_irqs,
            presented: 0,
            timeouts: 0,
            convert_us: 0,
            max_convert_us: 0,
            commit_us: 0,
        }
    }

    fn add(&mut self, convert_us: u64, commit_us: u64) {
        self.presented += 1;
        self.convert_us += convert_us;
        self.max_convert_us = self.max_convert_us.max(convert_us);
        self.commit_us += commit_us;
    }

    fn report(self, stats: esp_hal::mipi_csi::StreamStats, dsi_irqs: u32) -> Self {
        let elapsed_us = self.start.elapsed().as_micros().max(1);
        let per_s_x10 = |n: u32| n as u64 * 10_000_000 / elapsed_us;
        let presented = self.presented.max(1) as u64;
        println!(
            "live: camera_fps_x10={} delivered_fps_x10={} shown_fps_x10={} overwritten={} errored={} resyncs={} timeouts={} csi_irqs_per_s={} dsi_irqs_per_s={} convert_ms avg={} max={} commit_ms avg={} convert_cpu_pct={}",
            per_s_x10(stats.completed - self.stats.completed),
            per_s_x10(stats.delivered - self.stats.delivered),
            per_s_x10(self.presented),
            stats.overwritten - self.stats.overwritten,
            stats.errored - self.stats.errored,
            stats.resyncs - self.stats.resyncs,
            self.timeouts,
            per_s_x10(stats.interrupts - self.stats.interrupts) / 10,
            per_s_x10(dsi_irqs.wrapping_sub(self.dsi_irqs)) / 10,
            self.convert_us / presented / 1000,
            self.max_convert_us / 1000,
            self.commit_us / presented / 1000,
            self.convert_us * 100 / elapsed_us,
        );
        Self::new(stats, dsi_irqs)
    }
}

/// A zeroed PSRAM buffer that DMA can use.
fn alloc_dma(len: usize) -> &'static mut [u8] {
    let layout = Layout::from_size_align(len, DMA_ALIGNMENT).unwrap();
    let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
    assert!(!ptr.is_null(), "PSRAM allocation failed");
    unsafe { core::slice::from_raw_parts_mut(ptr, len) }
}

fn draw_color_bars(framebuffer: &mut [u8]) {
    const BARS: [(u8, u8, u8); 8] = [
        (255, 255, 255),
        (255, 255, 0),
        (0, 255, 255),
        (0, 255, 0),
        (255, 0, 255),
        (255, 0, 0),
        (0, 0, 255),
        (0, 0, 0),
    ];
    let lut = Rgb565Lut::new([GAIN_ONE; 3]);
    for row in framebuffer.as_chunks_mut::<{ WIDTH * 2 }>().0 {
        for (x, px) in row.as_chunks_mut::<2>().0.iter_mut().enumerate() {
            let (r, g, b) = BARS[x * BARS.len() / WIDTH];
            *px = lut.pack(r, g, b).to_le_bytes();
        }
    }
}

fn checksum32(data: &[u8]) -> u32 {
    data.as_chunks::<4>().0.iter().fold(0, |sum, &word| {
        sum.rotate_left(1) ^ u32::from_le_bytes(word)
    })
}

//! Captures from the SC2336 camera over MIPI-CSI while the EK79007 panel of the
//! ESP32-P4 Function EV Board streams a colour wheel over MIPI-DSI.
//!
//! DSI scans out from PSRAM on `VDMA_CH0`, and CSI captures into PSRAM on
//! `VDMA_CH1`. Both share the DW-GDMA interrupt and the MIPI D-PHY rail. The
//! run has four phases:
//!
//! 1. The colour wheel alone, as the reference.
//! 2. One `receive` call per frame. The DSI DMA must advance during every capture.
//! 3. A `CsiStream`, with one colour-wheel step between frames.
//! 4. Drop CSI, then check that DSI keeps streaming. Drop DSI, then check that the VDMA clock is
//!    off.
//!
//! The DSI bridge must never run out of pixels.
//!
//! Selected at build time:
//! - `CSI_FRAMES` (default 30): frames of phase 2 and phase 3.
//! - `CSI_COMPLETION`: `poll` (default) or `interrupt`, the receive call of phase 2.
//! - `CSI_ASYNC=1`: phase 3 takes the frames with `next_frame_async`.
//!
//! The panel uses LCD reset GPIO27 and backlight GPIO26. The run ends with a
//! `QA_RESULT: PASS` or `QA_RESULT: FAIL` line.
//% CHIP_FILTER: mipi_csi_driver_supported && mipi_dsi_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

extern crate alloc;

use core::{alloc::Layout, pin::pin};

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    mipi_csi::MipiCsi,
    mipi_dsi::{
        Config as DsiConfig,
        DataLanes as DsiDataLanes,
        MipiDsi,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, DsiDpi, FrameTiming},
    },
    peripherals::{GPIO26, GPIO27, HP_SYS_CLKRST, MIPI_DSI, MIPI_DSI_BRIDGE, VDMA, VDMA_CH0},
    psram,
    time::{Duration, Instant},
};
use esp_println::println;
use qa_test::camera::{
    self,
    FRAME_BYTES,
    GUARD_BYTES,
    PSRAM_DMA_ALIGNMENT,
    Sc2336,
    fill_sentinel,
    poll_on_wake,
    sentinel_words,
};

esp_bootloader_esp_idf::esp_app_desc!();

const LCD_H_ACTIVE: u32 = 1024;
const LCD_V_ACTIVE: u32 = 600;
const LCD_FB_BYTES: usize = LCD_H_ACTIVE as usize * LCD_V_ACTIVE as usize * 2;
/// About six frames at the panel's 58.8 Hz.
const LCD_VSYNC_TIMEOUT: Duration = Duration::from_millis(100);
const LCD_BASELINE_STEPS: u32 = 60;
const LCD_TAIL: Duration = Duration::from_millis(500);
const FRAME_TIMEOUT: Duration = Duration::from_millis(500);

/// Read back from `INTSIGNAL_ENABLE0` and `INTSTATUS_ENABLE0` after writing 0:
/// the bits the PAC leaves undefined plus bit 25. None of them latch a status.
const VDMA_ENABLE_STUCK_AT_ONE: u32 = 0x07c0_8004;

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

/// The colour wheel of `lcd-mipi-dsi`.
struct Lcd {
    dpi: DsiDpi<'static>,
    hue: u32,
    vsync_timeouts: u32,
    _reset: Output<'static>,
    _backlight: Output<'static>,
}

impl Lcd {
    fn start(
        dsi: MIPI_DSI<'static>,
        channel: VDMA_CH0<'static>,
        reset: GPIO27<'static>,
        backlight: GPIO26<'static>,
    ) -> Self {
        let delay = Delay::new();
        let mut reset = Output::new(reset, Level::Low, OutputConfig::default());
        delay.delay_millis(10);
        reset.set_high();
        delay.delay_millis(120);
        let backlight = Output::new(backlight, Level::High, OutputConfig::default());

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
        let fbs: [&'static mut [u8]; 2] = [allocate(), allocate()];
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
        MIPI_DSI_BRIDGE::regs()
            .int_clr()
            .write(|w| w.underrun().clear_bit_by_one());

        Self {
            dpi,
            hue: 0,
            vsync_timeouts: 0,
            _reset: reset,
            _backlight: backlight,
        }
    }

    /// One colour-wheel step, with a bounded vsync wait.
    fn render_step(&mut self) {
        let bridge = MIPI_DSI_BRIDGE::regs();
        let start = Instant::now();
        while !bridge.int_raw().read().vsync().bit_is_set() {
            if start.elapsed() > LCD_VSYNC_TIMEOUT {
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

    /// Frames the DSI DMA sent to the bridge.
    fn frames_sent(&self) -> u32 {
        self.dpi.dma_interrupt_count()
    }

    /// Whether the bridge ran out of pixels at any time since the start.
    fn underrun(&self) -> bool {
        MIPI_DSI_BRIDGE::regs()
            .int_raw()
            .read()
            .underrun()
            .bit_is_set()
    }

    /// Renders for `steps` steps or until `duration` passes, and returns the
    /// DSI frames sent and the vsync timeouts.
    fn run(&mut self, steps: u32, duration: Duration) -> (u32, u32) {
        let (frames_before, timeouts_before) = (self.frames_sent(), self.vsync_timeouts);
        let start = Instant::now();
        for _ in 0..steps {
            if start.elapsed() > duration {
                break;
            }
            self.render_step();
        }
        (
            self.frames_sent().wrapping_sub(frames_before),
            self.vsync_timeouts - timeouts_before,
        )
    }
}

fn hue_to_rgb565(hue: u32) -> u16 {
    let frac = hue % 60;
    let rise = (frac * 255 / 60) as u8;
    let fall = 255 - rise;
    let (r, g, b) = match hue / 60 {
        0 => (255, rise, 0),
        1 => (fall, 255, 0),
        2 => (0, 255, rise),
        3 => (0, fall, 255),
        4 => (rise, 0, 255),
        _ => (255, 0, fall),
    };
    ((r as u16) >> 3) << 11 | ((g as u16) >> 2) << 5 | (b as u16) >> 3
}

/// Signal-enable bits of `channel` that a driver set and that can raise the
/// interrupt.
fn routable_signal_enable(channel: usize) -> u32 {
    let ch = VDMA::regs().ch(channel);
    ch.intsignal_enable0().read().bits()
        & ch.intstatus_enable0().read().bits()
        & !VDMA_ENABLE_STUCK_AT_ONE
}

fn check(label: &str, ok: bool, detail: core::fmt::Arguments<'_>) -> bool {
    println!("{label}: {} {detail}", if ok { "PASS" } else { "FAIL" });
    ok
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    let frames: u32 = option_env!("CSI_FRAMES").map_or(30, |n| n.parse().unwrap());
    let interrupt = match option_env!("CSI_COMPLETION") {
        None | Some("poll") => false,
        Some("interrupt") => true,
        Some(other) => panic!("unknown CSI_COMPLETION '{other}', use poll or interrupt"),
    };
    let use_async = option_env!("CSI_ASYNC") == Some("1");
    println!("coexist: frames={frames} interrupt={interrupt} async={use_async}");

    let mut lcd = Lcd::start(
        peripherals.MIPI_DSI,
        peripherals.VDMA_CH0,
        peripherals.GPIO27,
        peripherals.GPIO26,
    );
    let (sent, timeouts) = lcd.run(LCD_BASELINE_STEPS, Duration::from_secs(5));
    let mut ok = check(
        "lcd-baseline",
        sent > 0 && timeouts == 0,
        format_args!("steps={LCD_BASELINE_STEPS} dsi_frames={sent} vsync_timeouts={timeouts}"),
    );

    let mut capture = MipiCsi::new(peripherals.MIPI_CSI, camera::csi_config())
        .unwrap()
        .with_dma(peripherals.VDMA_CH1, camera::capture_config())
        .unwrap();
    let mut sensor = Sc2336::new(
        peripherals.I2C0,
        peripherals.GPIO7,
        peripherals.GPIO8,
        false,
    );

    let len = 2 * (FRAME_BYTES + GUARD_BYTES);
    let layout = Layout::from_size_align(len, PSRAM_DMA_ALIGNMENT).unwrap();
    let region = unsafe { alloc::alloc::alloc(layout) };
    assert!(!region.is_null(), "frame buffer allocation failed");
    let region = unsafe { core::slice::from_raw_parts_mut(region, len) };
    let (first, second) = region.split_at_mut(FRAME_BYTES + GUARD_BYTES);
    let (buffer0, guard0) = first.split_at_mut(FRAME_BYTES);
    let (buffer1, guard1) = second.split_at_mut(FRAME_BYTES);
    let mut guards = [guard0, guard1];
    guards.iter_mut().for_each(|guard| fill_sentinel(guard));

    // Phase 2: one receive call per frame, each with the sensor started and
    // stopped around it.
    let (mut valid, mut without_dsi_progress) = (0, 0);
    for index in 1..=frames {
        if index % 10 == 0 {
            println!("receive[{index}/{frames}]: valid={valid}");
        }
        lcd.render_step();
        fill_sentinel(buffer0);
        let sent_before = lcd.frames_sent();
        let start_sensor = || sensor.set_stream(true).unwrap();
        let result = if interrupt {
            capture.receive_with_interrupt(buffer0, start_sensor)
        } else {
            capture.receive_with(buffer0, start_sensor)
        };
        without_dsi_progress += u32::from(lcd.frames_sent() == sent_before);
        sensor.set_stream(false).unwrap();
        let report = capture.completion_interrupt_report();
        if result.is_ok()
            && sentinel_words(buffer0) == 0
            && (!interrupt || (report.fired && report.own_source))
        {
            valid += 1;
        } else {
            println!("receive: result={result:?} completion_irq={report:?}");
        }
    }
    ok &= check(
        "receive",
        valid == frames && without_dsi_progress == 0,
        format_args!("valid={valid}/{frames} captures_without_dsi_progress={without_dsi_progress}"),
    );

    // Phase 3: a stream with one colour-wheel step between frames.
    let sent_before = lcd.frames_sent();
    let stats;
    let mut delivered = 0;
    let stream_on;
    if use_async {
        let mut stream = capture
            .into_async()
            .into_stream([buffer0, buffer1])
            .map_err(|(_, _, error)| error)
            .expect("stream did not start");
        stream_on = sensor.set_stream(true);
        for _ in 0..frames {
            lcd.render_step();
            match poll_on_wake(pin!(stream.next_frame_async()), FRAME_TIMEOUT) {
                Some(Ok(_)) => delivered += 1,
                other => {
                    println!("stream: {:?}", other.map(|r| r.map(|f| f.sequence())));
                    break;
                }
            }
        }
        stats = stream.stats();
        capture = stream.stop().0.into_blocking();
    } else {
        let mut stream = capture
            .into_stream([buffer0, buffer1])
            .map_err(|(_, _, error)| error)
            .expect("stream did not start");
        stream_on = sensor.set_stream(true);
        for _ in 0..frames {
            lcd.render_step();
            match stream.next_frame(FRAME_TIMEOUT) {
                Ok(_) => delivered += 1,
                Err(error) => {
                    println!("stream: {error:?}");
                    break;
                }
            }
        }
        stats = stream.stats();
        capture = stream.stop().0;
    }
    let stream_off = sensor.set_stream(false);
    let sent = lcd.frames_sent().wrapping_sub(sent_before);
    let guards_clean = guards
        .iter_mut()
        .all(|guard| sentinel_words(guard) == GUARD_BYTES / 4);
    ok &= check(
        "stream",
        delivered == frames
            && stats.errored == 0
            && sent > 0
            && guards_clean
            && stream_on.is_ok()
            && stream_off.is_ok(),
        format_args!(
            "delivered={delivered}/{frames} {stats:?} dsi_frames={sent} guards_clean={guards_clean}"
        ),
    );

    // Phase 4: drop CSI, then DSI, and check that each leaves the other, and
    // no interrupt state, behind.
    drop(capture);
    let vdma = VDMA::regs();
    let chen = vdma.chen0().read().bits();
    let csi_released = chen & 0b10 == 0
        && routable_signal_enable(1) == 0
        && vdma.ch(1).intstatus0().read().bits() == 0;
    let (sent, timeouts) = lcd.run(u32::MAX, LCD_TAIL);
    ok &= check(
        "csi-drop",
        csi_released && chen & 0b01 != 0 && sent > 0 && timeouts == 0,
        format_args!(
            "csi_channel_released={csi_released} chen0={:#x} dsi_frames_after={sent} vsync_timeouts={timeouts}",
            chen & 0xf
        ),
    );
    let underrun = lcd.underrun();
    let vsync_timeouts = lcd.vsync_timeouts;
    drop(lcd);
    // With the clock gated no channel can raise the interrupt.
    let vdma_clock = HP_SYS_CLKRST::regs()
        .soc_clk_ctrl1()
        .read()
        .gdma_sys_clk_en()
        .bit();
    ok &= check(
        "dsi-drop",
        !vdma_clock && !underrun && vsync_timeouts == 0,
        format_args!(
            "vdma_clock_enabled={vdma_clock} dsi_underrun={underrun} vsync_timeouts={vsync_timeouts}"
        ),
    );

    println!(
        "QA_RESULT: {} test=csi_dsi_coexist frames={frames} interrupt={interrupt} async={use_async}",
        if ok { "PASS" } else { "FAIL" }
    );
    loop {}
}

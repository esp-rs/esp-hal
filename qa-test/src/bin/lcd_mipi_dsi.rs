//! Drives the EK79007 MIPI-DSI panel on the ESP32-P4-Function-EV-Board v1.5.
//!
//! Cycles through a smooth RGB colour wheel one frame per vsync.
//!
//! Board wiring (fixed, no external GPIOs needed):
//! - LCD RST     => GPIO27
//! - LCD BL PWM  => GPIO26  (driven high = backlight on)
//! - MIPI-DSI    => internal (no GPIO mux)
//! - VDD_MIPI_DPHY powered by the driver via PMU LDO3

//% CHIP_FILTER: mipi_dsi_driver_supported

#![no_std]
#![no_main]

extern crate alloc;

use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::{
        CpuClock,
        ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    },
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    mipi_dsi::{
        Config,
        DataLanes,
        MipiDsi,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, FrameTiming},
    },
    peripherals::Peripherals,
    psram,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// ── Panel constants (EK79007, 1024×600 @ 60 Hz) ──────────────────────────────

const H_ACTIVE: u32 = 1024;
const V_ACTIVE: u32 = 600;
const BYTES_PER_PIXEL: usize = 2; // RGB565
const FB_SIZE: usize = H_ACTIVE as usize * V_ACTIVE as usize * BYTES_PER_PIXEL;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals: Peripherals = esp_hal::init(config);

    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    println!("PSRAM ready");

    let delay = Delay::new();

    // ── LCD reset (GPIO27, active-low) ──────────────────────────────────────
    let mut lcd_rst = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());
    delay.delay_millis(10);
    lcd_rst.set_high();
    delay.delay_millis(120);

    // ── Backlight on (GPIO26, active-high) ─────────────────────────────────
    let _lcd_bl = Output::new(peripherals.GPIO26, Level::High, OutputConfig::default());

    // ── MIPI DSI bus ────────────────────────────────────────────────────────
    let mut bus = MipiDsi::new(
        peripherals.MIPI_DSI,
        peripherals.VDMA_CH0,
        Config::default()
            .with_num_data_lanes(DataLanes::_2)
            .with_lane_bit_rate_mbps(1000.0)
            .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                MipiDsiPhyPllRefclkSclk::Xtal,
                0,
            ))
            .with_force_clock_lane_hs(false),
    )
    .expect("MipiDsi init failed");

    println!("DSI bus up");

    // ── EK79007 init via DBI (LP command mode) ──────────────────────────────
    {
        let mut dbi = bus.dbi(0);
        for &(cmd, params) in EK79007_INIT {
            dbi.write_cmd(cmd, params).unwrap();
        }
        // Sleep out: 120 ms settle time.
        dbi.write_cmd(0x11, &[]).unwrap();
        delay.delay_millis(120);
        // Display on.
        dbi.write_cmd(0x29, &[]).unwrap();
        delay.delay_millis(20);
    }

    println!("Panel init done");

    // ── Frame buffer in PSRAM (64-byte aligned) ─────────────────────────────
    let layout = Layout::from_size_align(FB_SIZE, 64).unwrap();
    let fb_ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
    assert!(!fb_ptr.is_null(), "PSRAM alloc failed");
    let fb1: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(fb_ptr, FB_SIZE) };

    let fb_ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
    assert!(!fb_ptr.is_null(), "PSRAM alloc failed");
    let fb2: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(fb_ptr, FB_SIZE) };
    let fbs: [&mut [u8]; 2] = [fb1, fb2];

    // ── Enter video mode (DPI) ──────────────────────────────────────────────
    let dpi_cfg = DpiConfig {
        virtual_channel: 0,
        pixel_clock_mhz: 48.0,
        dpi_clk_src: DpiClockSource::PllF240m,
        in_color_format: ColorFormat::Rgb565,
        out_color_format: ColorFormat::Rgb565,
        timing: FrameTiming {
            h_active: H_ACTIVE,
            hsw: 10,
            hbp: 120,
            hfp: 120,
            v_active: V_ACTIVE,
            vsw: 1,
            vbp: 20,
            vfp: 20,
        },
    };

    let mut dpi = bus.dpi(dpi_cfg, &fbs).expect("DPI init failed");

    println!("Streaming");

    // ── Colour wheel loop ───────────────────────────────────────────────────
    let mut hue: u32 = 0;

    loop {
        // Pace to the display refresh rate.  The DMA streams the front buffer
        // continuously; this just waits for the blanking pulse so we know the
        // frame we are about to render will be displayed for exactly one vsync
        // period.
        dpi.wait_for_vsync();

        let back = dpi.framebuffer_mut();

        // Generate color
        let px = hue_to_rgb565(hue);

        // Fill the framebuffer with the color, 16 bytes at a time.
        let w = px as u32 | (px as u32) << 16;
        let c = w as u128 | (w as u128) << 32 | (w as u128) << 64 | (w as u128) << 96;
        for chunk in back.chunks_exact_mut(16) {
            chunk.copy_from_slice(&c.to_le_bytes());
        }

        // Flush cache and redirect DMA to the new buffer.
        // The DMA never stops; the change appears within one block transfer.
        dpi.commit();

        hue = (hue + 1) % 360;
    }
}

// ── Colour wheel ──────────────────────────────────────────────────────────────

/// Convert a hue value (0–359°) to a packed RGB565 value.
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
    let r5 = (r as u16) >> 3;
    let g6 = (g as u16) >> 2;
    let b5 = (b as u16) >> 3;
    (r5 << 11) | (g6 << 5) | b5
}

// ── EK79007 vendor init sequence ─────────────────────────────────────────────

static EK79007_INIT: &[(u8, &[u8])] = &[
    (0xB2, &[0x10]), // Pad control - two lanes
    (0x80, &[0x8B]),
    (0x81, &[0x78]),
    (0x82, &[0x84]),
    (0x83, &[0x88]),
    (0x84, &[0xA8]),
    (0x85, &[0xE3]),
    (0x86, &[0x88]),
];

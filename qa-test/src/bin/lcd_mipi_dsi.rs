//! Drives the EK79007 MIPI-DSI panel on the ESP32-P4-Function-EV-Board v1.5.
//!
//! Cycles through a smooth RGB colour wheel one frame per vsync.
//!
//! Board wiring (fixed, no external GPIOs needed):
//! - LCD RST     => GPIO27
//! - LCD BL PWM  => GPIO26  (driven high = backlight on)
//! - MIPI-DSI    => internal (no GPIO mux)
//! - LDO3 (VO3) supplies VDD_MIPI_DPHY at 2.5 V via the internal PMU

//% CHIP_FILTER: mipi_dsi_driver_supported

#![no_std]
#![no_main]

extern crate alloc;

use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    mipi_dsi::{
        Config, DataLanes, MipiDsi, PhyPllClockSource,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, FrameTiming},
    },
    peripherals::{PMU, Peripherals},
    psram,
    rom,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// ── Panel constants (EK79007, 1024×600 @ 60 Hz) ──────────────────────────────

const H_ACTIVE: u32 = 1024;
const V_ACTIVE: u32 = 600;
const BYTES_PER_PIXEL: usize = 2; // RGB565
const FB_SIZE: usize = H_ACTIVE as usize * V_ACTIVE as usize * BYTES_PER_PIXEL;

// ── DPHY LDO constants ────────────────────────────────────────────────────────
// LDO channel 3 (VO3) powers VDD_MIPI_DPHY at 2.5 V.
// Mapping: chan_id=3 → ldo_unit=2 → PMU.ext_ldo[1] → p0_0p2a registers.
//
// Without eFuse calibration (K=1, Vos=0, C=1):
//   Vref = 1.0 V  (dref=9 → Vref=1+(9-9)*0.1=1.0)
//   Vout = Vref*(1+0.25*mul) = 1.0*(1+1.5) = 2.5 V  (mul=6)
//
// ctrl  bits: force_tieh_sel=1[7], xpd=1[8], target0=0x80[30:23], target1=0x40[22:15]
// ana   bits: dref=9[31:28], mul=6[25:23]
const DPHY_LDO_CTRL: u32 = 0x4020_0180; // enable + SW-owned
const DPHY_LDO_ANA: u32 = (9u32 << 28) | (6u32 << 23); // 2.5 V

#[main]
fn main() -> ! {
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    println!("PSRAM ready");

    enable_dphy_ldo();
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
        peripherals.VDMA,
        Config {
            num_data_lanes:    DataLanes::_2,
            lane_bit_rate_mbps: 1000.0,
            phy_clk_src:       PhyPllClockSource::Xtal,
            force_clock_lane_hs: false,
        },
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
    let fb: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(fb_ptr, FB_SIZE) };

    // ── Enter video mode (DPI) ──────────────────────────────────────────────
    let dpi_cfg = DpiConfig {
        virtual_channel:  0,
        pixel_clock_mhz:  48.0,
        dpi_clk_src:      DpiClockSource::PllF240m,
        in_color_format:  ColorFormat::Rgb565,
        out_color_format: ColorFormat::Rgb565,
        timing: FrameTiming {
            h_active: H_ACTIVE,
            hsw:      10,
            hbp:      120,
            hfp:      120,
            v_active: V_ACTIVE,
            vsw:      1,
            vbp:      20,
            vfp:      10,
        },
    };

    let fbs: [&mut [u8]; 1] = [fb];
    let mut dpi = bus
        .dpi(dpi_cfg, &fbs)
        .expect("DPI init failed");

    println!("Streaming");

    // ── Colour wheel loop ───────────────────────────────────────────────────
    let mut hue: u32 = 0;

    loop {
        dpi.wait_for_vsync();

        let back = dpi.framebuffer_mut();
        let rgb565 = hue_to_rgb565(hue);
        for chunk in back.chunks_exact_mut(2) {
            chunk.copy_from_slice(&rgb565.to_be_bytes());
        }
        dpi.commit();

        hue = (hue + 1) % 360;
    }
}

// ── LDO helpers ───────────────────────────────────────────────────────────────

fn enable_dphy_ldo() {
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(DPHY_LDO_CTRL) });
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits(DPHY_LDO_ANA) });
    // Allow LDO output to settle.
    rom::ets_delay_us(500);
}

// ── Colour wheel ──────────────────────────────────────────────────────────────

/// Convert a hue value (0–359°) to a packed RGB565 big-endian word.
fn hue_to_rgb565(hue: u32) -> u16 {
    let region = hue / 60;
    let frac   = hue % 60;
    let (r, g, b) = match region {
        0 => (255,              (frac * 255 / 60) as u8, 0),
        1 => ((255 - frac * 255 / 60) as u8, 255, 0),
        2 => (0, 255,          (frac * 255 / 60) as u8),
        3 => (0, (255 - frac * 255 / 60) as u8, 255),
        4 => ((frac * 255 / 60) as u8, 0, 255),
        _ => (255, 0,          (255 - frac * 255 / 60) as u8),
    };
    let r5 = (r as u16) >> 3;
    let g6 = (g as u16) >> 2;
    let b5 = (b as u16) >> 3;
    (r5 << 11) | (g6 << 5) | b5
}

// ── EK79007 vendor init sequence ─────────────────────────────────────────────

static EK79007_INIT: &[(u8, &[u8])] = &[
    (0x80, &[0x8B]),
    (0x81, &[0x78]),
    (0x82, &[0x84]),
    (0x83, &[0x88]),
    (0x84, &[0xA8]),
    (0x85, &[0xE3]),
    (0x86, &[0x88]),
];

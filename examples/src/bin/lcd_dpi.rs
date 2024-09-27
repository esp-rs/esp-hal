//! Drives the 16-bit parallel RGB display on Makerfabs ESP32-S3-Parallel-TFT-with-Touch-4.3inch
//!
//! This example fills the screen with every color.
//!
//! The following wiring is assumed:
//! - LCD_VYSNC  => GPIO41
//! - LCD_HSYNC  => GPIO39
//! - LCD_DE     => GPIO40
//! - LCD_PCLK   => GPIO42
//! - LCD_DATA0  => GPIO8
//! - LCD_DATA1  => GPIO3
//! - LCD_DATA2  => GPIO46
//! - LCD_DATA3  => GPIO9
//! - LCD_DATA4  => GPIO1
//! - LCD_DATA5  => GPIO5
//! - LCD_DATA6  => GPIO6
//! - LCD_DATA7  => GPIO7
//! - LCD_DATA8  => GPIO15
//! - LCD_DATA9  => GPIO16
//! - LCD_DATA10 => GPIO4
//! - LCD_DATA11 => GPIO45
//! - LCD_DATA12 => GPIO48
//! - LCD_DATA13 => GPIO47
//! - LCD_DATA14 => GPIO21
//! - LCD_DATA15 => GPIO14

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use core::iter::empty;

use esp_backtrace as _;
use esp_hal::{
    dma::{Dma, DmaPriority},
    dma_loop_buffer,
    gpio::{Io, Level},
    lcd_cam::{
        lcd::{
            dpi::{Config, Dpi, Format, FrameTiming},
            ClockMode,
            Phase,
            Polarity,
        },
        LcdCam,
    },
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    let channel = dma.channel2.configure(true, DmaPriority::Priority0);
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let mut dma_buf = dma_loop_buffer!(2 * 16);

    let config = Config {
        clock_mode: ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        },
        format: Format {
            enable_2byte_mode: true,
            ..Default::default()
        },
        // https://www.makerfabs.com/desfile/files/QT4300H40R10-V03-Spec.pdf
        timing: FrameTiming {
            horizontal_active_width: 800,
            horizontal_total_width: 928,      // 889..1143
            horizontal_blank_front_porch: 40, // 1..255

            vertical_active_height: 480,
            vertical_total_height: 525,     // 513..767
            vertical_blank_front_porch: 13, // 1..255

            hsync_width: 48, // 1..255
            vsync_width: 3,  // 3..255

            hsync_position: 0,
        },
        vsync_idle_level: Level::High,
        hsync_idle_level: Level::High,
        de_idle_level: Level::Low,
        disable_black_region: false,
        ..Default::default()
    };

    // https://raw.githubusercontent.com/Makerfabs/ESP32-S3-Parallel-TFT-with-Touch-4.3inch/main/hardware/ESP32-S3%20Parallel%20TFT%20with%20Touch%204.3%E2%80%9C%20V3.1.PDF
    let mut dpi = Dpi::new(lcd_cam.lcd, channel.tx, 30.MHz(), config)
        .with_ctrl_pins(
            io.pins.gpio41,
            io.pins.gpio39,
            io.pins.gpio40,
            io.pins.gpio42,
        )
        .with_data_pins(
            // Blue
            io.pins.gpio8,
            io.pins.gpio3,
            io.pins.gpio46,
            io.pins.gpio9,
            io.pins.gpio1,
            // Green
            io.pins.gpio5,
            io.pins.gpio6,
            io.pins.gpio7,
            io.pins.gpio15,
            io.pins.gpio16,
            io.pins.gpio4,
            // Red
            io.pins.gpio45,
            io.pins.gpio48,
            io.pins.gpio47,
            io.pins.gpio21,
            io.pins.gpio14,
        );

    const MAX_RED: u16 = (1 << 5) - 1;
    const MAX_GREEN: u16 = (1 << 6) - 1;
    const MAX_BLUE: u16 = (1 << 5) - 1;

    fn rgb(r: u16, g: u16, b: u16) -> u16 {
        (r << 11) | (g << 5) | (b << 0)
    }

    let mut colors = empty()
        // Start with red and gradually add green
        .chain((0..=MAX_GREEN).map(|g| rgb(MAX_RED, g, 0)))
        // Then remove the red
        .chain((0..=MAX_RED).rev().map(|r| rgb(r, MAX_GREEN, 0)))
        // Then add blue
        .chain((0..=MAX_BLUE).map(|b| rgb(0, MAX_GREEN, b)))
        // Then remove green
        .chain((0..=MAX_GREEN).rev().map(|g| rgb(0, g, MAX_BLUE)))
        // Then add red
        .chain((0..=MAX_RED).map(|r| rgb(r, 0, MAX_BLUE)))
        // Then remove blue
        .chain((0..=MAX_BLUE).rev().map(|b| rgb(MAX_RED, 0, b)))
        // Once we get we have red, and we can start again.
        .cycle();

    println!("Rendering");
    loop {
        let transfer = dpi.send(false, dma_buf).map_err(|e| e.0).unwrap();
        (_, dpi, dma_buf) = transfer.wait();

        if let Some(color) = colors.next() {
            for chunk in dma_buf.chunks_mut(2) {
                chunk.copy_from_slice(&color.to_le_bytes());
            }
        }
    }
}

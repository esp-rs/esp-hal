//! Drives the 16-bit parallel RGB (DPI) display on PandaTouch v1.x using
//! a PSRAM framebuffer with GDMA bounce buffer pipeline.
//!
//! The following wiring is assumed (PandaTouch BSP pinout):
//! - LCD_PCLK   => GPIO5
//! - LCD_DE     => GPIO38
//! - LCD_RST    => GPIO46
//! - LCD_BL     => GPIO21
//! - LCD_DATA0  => GPIO17  (B3)
//! - LCD_DATA1  => GPIO18  (B4)
//! - LCD_DATA2  => GPIO48  (B5)
//! - LCD_DATA3  => GPIO47  (B6)
//! - LCD_DATA4  => GPIO39  (B7)
//! - LCD_DATA5  => GPIO11  (G2)
//! - LCD_DATA6  => GPIO12  (G3)
//! - LCD_DATA7  => GPIO13  (G4)
//! - LCD_DATA8  => GPIO14  (G5)
//! - LCD_DATA9  => GPIO15  (G6)
//! - LCD_DATA10 => GPIO16  (G7)
//! - LCD_DATA11 => GPIO6   (R3)
//! - LCD_DATA12 => GPIO7   (R4)
//! - LCD_DATA13 => GPIO8   (R5)
//! - LCD_DATA14 => GPIO9   (R6)
//! - LCD_DATA15 => GPIO10  (R7)

//% CHIPS: esp32s3
//% FEATURES: esp-hal/unstable esp-hal/psram
//% ENV: ESP_HAL_CONFIG_PSRAM_MODE=octal

#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{DmaBounceBuffer, DmaDescriptor},
    gpio::{Level, Output, OutputConfig},
    lcd_cam::{
        LcdCam,
        lcd::{
            ClockMode,
            Phase,
            Polarity,
            dpi::{Config, Dpi, Format, FrameTiming},
        },
    },
    main,
    peripherals::Peripherals,
    time::Rate,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// SAFETY: Each static mut below is passed exactly once into DmaBounceBuffer,
// which becomes the sole owner via install() into the global ISR critical section.
static mut BOUNCE_DESCRIPTORS: [DmaDescriptor; 2] = [DmaDescriptor::EMPTY; 2];

const BOUNCE_BUF_SIZE: usize = 10 * 800 * 2; // 10 lines × 800 px × 2 bytes
static mut BOUNCE_BUF0: [u8; BOUNCE_BUF_SIZE] = [0u8; BOUNCE_BUF_SIZE];
static mut BOUNCE_BUF1: [u8; BOUNCE_BUF_SIZE] = [0u8; BOUNCE_BUF_SIZE];

fn rgb565(r: u16, g: u16, b: u16) -> u16 {
    (r << 11) | (g << 5) | b
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    let mut rst = Output::new(peripherals.GPIO46, Level::Low, OutputConfig::default());
    delay.delay_millis(100u32);
    rst.set_high();
    delay.delay_millis(100u32);

    let _backlight = Output::new(peripherals.GPIO21, Level::High, OutputConfig::default());

    let channel = peripherals.DMA_CH0;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    const FB_SIZE: usize = 800 * 480 * 2;
    let mut fb0_vec: Vec<u8> = Vec::with_capacity(FB_SIZE);
    fb0_vec.resize(FB_SIZE, 0u8);
    let framebuffer0: &'static mut [u8] = fb0_vec.leak();

    let mut fb1_vec: Vec<u8> = Vec::with_capacity(FB_SIZE);
    fb1_vec.resize(FB_SIZE, 0u8);
    let framebuffer1: &'static mut [u8] = fb1_vec.leak();

    let blue = rgb565(0, 0, 31).to_le_bytes();
    for chunk in framebuffer0.chunks_exact_mut(2) {
        chunk.copy_from_slice(&blue);
    }

    let config = Config::default()
        .with_frequency(Rate::from_mhz(23))
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            // pclk_active_neg = true  →  data shifts on rising edge,
            // clock idle is low, so output at falling edge = ShiftHigh.
            phase: Phase::ShiftHigh,
        })
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            // Total = active + hsync_pulse_width + hbp + hfp
            // 800 + 4 + 8 + 8 = 820
            horizontal_total_width: 820,
            horizontal_active_width: 800,
            // blank_front_porch includes hsync_width per esp-hal docs: hsw + hbp = 4 + 8 = 12
            horizontal_blank_front_porch: 12,
            // Total = active + vsync_pulse_width + vbp + vfp
            // 480 + 4 + 16 + 16 = 516
            vertical_total_height: 516,
            vertical_active_height: 480,
            // vsync_width + vbp = 4 + 16 = 20
            vertical_blank_front_porch: 20,
            hsync_width: 4,
            vsync_width: 4,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let dpi = Dpi::new(lcd_cam.lcd, channel, config)
        .unwrap()
        .with_de(peripherals.GPIO38)
        .with_pclk(peripherals.GPIO5)
        // Blue B3-B7
        .with_data0(peripherals.GPIO17)
        .with_data1(peripherals.GPIO18)
        .with_data2(peripherals.GPIO48)
        .with_data3(peripherals.GPIO47)
        .with_data4(peripherals.GPIO39)
        // Green G2-G7
        .with_data5(peripherals.GPIO11)
        .with_data6(peripherals.GPIO12)
        .with_data7(peripherals.GPIO13)
        .with_data8(peripherals.GPIO14)
        .with_data9(peripherals.GPIO15)
        .with_data10(peripherals.GPIO16)
        // Red R3-R7
        .with_data11(peripherals.GPIO6)
        .with_data12(peripherals.GPIO7)
        .with_data13(peripherals.GPIO8)
        .with_data14(peripherals.GPIO9)
        .with_data15(peripherals.GPIO10);

    // SAFETY: Each static mut is accessed only once here, before the bounce
    // buffer state is installed into the global ISR slot. After install(),
    // DmaBounceBuffer is the sole owner of these regions.
    let bounce_state = unsafe {
        DmaBounceBuffer::new(
            &mut *core::ptr::addr_of_mut!(BOUNCE_DESCRIPTORS),
            framebuffer0,
            core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(BOUNCE_BUF0).cast::<u8>(),
                BOUNCE_BUF_SIZE,
            ),
            core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(BOUNCE_BUF1).cast::<u8>(),
                BOUNCE_BUF_SIZE,
            ),
        )
        .expect("bounce buffer setup failed")
    };

    println!("Initialising");
    let mut transfer = dpi.send(true, bounce_state).map_err(|e| e.0).unwrap();
    transfer.set_back_buffer(framebuffer1);
    println!("Rendering");

    let colors: &[u16] = &[
        rgb565(31, 0, 0),
        rgb565(0, 63, 0),
        rgb565(0, 0, 31),
        rgb565(31, 63, 31),
        rgb565(0, 0, 0),
    ];

    let mut color_idx = 0usize;
    let mut next_change = esp_hal::time::Instant::now();
    loop {
        transfer.poll();

        if next_change <= esp_hal::time::Instant::now() {
            next_change = esp_hal::time::Instant::now() + esp_hal::time::Duration::from_secs(2);

            let draw = transfer.back_buffer().unwrap();
            let bytes = colors[color_idx % colors.len()].to_le_bytes();
            for chunk in draw.chunks_exact_mut(2) {
                chunk.copy_from_slice(&bytes);
            }

            color_idx += 1;
            transfer.swap_buffers();
        }
    }
}

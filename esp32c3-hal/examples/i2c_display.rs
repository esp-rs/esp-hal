//! I2C Display example
//!
//! This example prints some text on an SSD1306-based
//! display (via I2C)
//!
//! The following wiring is assumed:
//! - SDA => GPIO2
//! - SCL => GPIO3

#![no_std]
#![no_main]

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X10, FONT_9X18_BOLD},
        MonoTextStyleBuilder,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use esp32c3_hal::{gpio::IO, i2c, pac::Peripherals, prelude::*, RtcCntl, Timer, I2C};
use nb::block;
use panic_halt as _;
use riscv_rt::entry;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);

    // Disable watchdog timers
    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Enable the I2C peripheral clock
    peripherals
        .SYSTEM
        .perip_clk_en0
        .modify(|_, w| w.ext0_clk_en().set_bit());

    // Take the I2C peripheral out of any pre-existing reset state
    // (shouldn't be the case after a fresh startup, but better be safe)
    peripherals
        .SYSTEM
        .perip_rst_en0
        .modify(|_, w| w.ext0_rst().clear_bit());

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new(
        peripherals.I2C,
        i2c::Pins {
            sda: io.pins.gpio2,
            scl: io.pins.gpio3,
        },
        100_000,
    )
    .unwrap();

    // Start timer (5 second interval)
    timer0.start(50_000_000u64);

    // Initialize display
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Specify different text styles
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_big = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    loop {
        // Fill display bufffer with a centered text with two lines (and two text
        // styles)
        Text::with_alignment(
            "esp-hal",
            display.bounding_box().center() + Point::new(0, 0),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_alignment(
            "Chip: ESP32-C3",
            display.bounding_box().center() + Point::new(0, 14),
            text_style,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
        display.clear();

        // Wait 5 seconds
        block!(timer0.wait()).unwrap();

        // Write single-line centered text "Hello World" to buffer
        Text::with_alignment(
            "Hello World!",
            display.bounding_box().center(),
            text_style_big,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
        display.clear();

        // Wait 5 seconds
        block!(timer0.wait()).unwrap();
    }
}

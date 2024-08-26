//! SPI loopback test
//!
//! The following wiring is assumed:
//! - SCLK => GPIO0
//! - MISO => GPIO2
//! - MOSI => GPIO4
//! - CS   => GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, Io},
    prelude::*,
    spi::{master::Spi, SpiMode},
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;
    let cs = io.pins.gpio5;

    let miso = AnyPin::new(miso);
    let mosi = AnyPin::new(mosi);

    let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sclk),
        Some(mosi),
        Some(miso),
        Some(cs),
    );

    let delay = Delay::new(&clocks);

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        spi.transfer(&mut data).unwrap();
        println!("{:x?}", data);

        delay.delay_millis(250);
    }
}

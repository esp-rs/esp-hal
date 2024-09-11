//! SPI loopback test
//!
//! The following wiring is assumed:
//! - SCLK => GPIO0
//! - MISO/MOSI => GPIO2
//! - CS   => GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    spi::{master::Spi, SpiMode},
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = peripherals.GPIO.pins();
    let sclk = io.gpio0;
    let miso_mosi = io.gpio2;
    let cs = io.gpio5;

    let miso = miso_mosi.peripheral_input();
    let mosi = miso_mosi.into_peripheral_output();

    let mut spi =
        Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0).with_pins(sclk, mosi, miso, cs);

    let delay = Delay::new();

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        spi.transfer(&mut data).unwrap();
        println!("{:x?}", data);

        delay.delay_millis(250);
    }
}

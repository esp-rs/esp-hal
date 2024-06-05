//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO4
//! CS      GPIO5
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
    clock::ClockControl,
    delay::Delay,
    gpio::{any_pin::AnyPin, Io},
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{Spi, SpiParts},
        SpiMode,
    },
    system::SystemControl,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;
    let cs = io.pins.gpio5;

    let miso = AnyPin::new(miso);
    let mosi = AnyPin::new(mosi);

    let SpiParts { spi, mut buf, .. } =
        Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks);
    let mut spi = spi.with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs));

    let delay = Delay::new(&clocks);

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        buf.write(&data);
        (spi, buf) = spi.transfer(data.len(), buf).unwrap().wait();
        buf.read(&mut data);
        println!("{:x?}", data);

        delay.delay_millis(250);
    }
}

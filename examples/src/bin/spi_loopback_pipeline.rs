//! SPI loopback test with pipelining
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

    let SpiParts { spi, buf, .. } = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks);
    let spi = spi.with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs));

    let (mut buf0, buf1) = buf.split();

    let delay = Delay::new(&clocks);

    let data_to_send = [0xDA, 0xB0, 0xDE, 0xCA, 0xFB, 0xAD, 0xBE, 0xEF, 0xAF, 0x22];

    buf0.write(&data_to_send);
    let mut active_transfer = spi.transfer(data_to_send.len(), buf0).unwrap();
    let mut available_buf = buf1;

    loop {
        available_buf.write(&data_to_send);
        let (spi, new_buf) = active_transfer.wait();
        active_transfer = spi.transfer(data_to_send.len(), available_buf).unwrap();
        available_buf = new_buf;

        let mut data_to_receive = [0; 10];
        available_buf.read(&mut data_to_receive);

        println!("{:x?}", data_to_receive);

        delay.delay_millis(250);
    }
}

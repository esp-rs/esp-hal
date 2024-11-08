//! SPI read manufacturer id from flash chip
//!
//! The following wiring is assumed:
//! - SCLK        =>  GPIO0
//! - MISO/IO0    =>  GPIO1
//! - MOSI/IO1    =>  GPIO2
//! - IO2         =>  GPIO3
//! - IO3         =>  GPIO4
//! - CS          =>  GPIO5
//!
//! The following wiring is assumed for ESP32:
//! - SCLK        =>  GPIO0
//! - MISO/IO0    =>  GPIO2
//! - MOSI/IO1    =>  GPIO4
//! - IO2         =>  GPIO5
//! - IO3         =>  GPIO13
//! - CS          =>  GPIO14
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect a flash chip (GD25Q64C was used) and make sure QE in the status
//! register is set.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    spi::{
        master::{Address, Command, Config, Spi},
        SpiDataMode,
        SpiMode,
    },
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let sclk = peripherals.pins.gpio0;
            let miso = peripherals.pins.gpio2;
            let mosi = peripherals.pins.gpio4;
            let sio2 = peripherals.pins.gpio5;
            let sio3 = peripherals.pins.gpio13;
            let cs = peripherals.pins.gpio14;
        } else {
            let sclk = peripherals.pins.gpio0;
            let miso = peripherals.pins.gpio1;
            let mosi = peripherals.pins.gpio2;
            let sio2 = peripherals.pins.gpio3;
            let sio3 = peripherals.pins.gpio4;
            let cs = peripherals.pins.gpio5;
        }
    }

    let mut spi = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 100.kHz(),
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    )
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_sio2(sio2)
    .with_sio3(sio3)
    .with_cs(cs);

    let delay = Delay::new();

    loop {
        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.half_duplex_read(
            SpiDataMode::Single,
            Command::Command8(0x90, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            &mut data,
        )
        .unwrap();
        println!("Single {:x?}", data);
        delay.delay_millis(250);

        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.half_duplex_read(
            SpiDataMode::Dual,
            Command::Command8(0x92, SpiDataMode::Single),
            Address::Address32(0x000000_00, SpiDataMode::Dual),
            0,
            &mut data,
        )
        .unwrap();
        println!("Dual {:x?}", data);
        delay.delay_millis(250);

        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.half_duplex_read(
            SpiDataMode::Quad,
            Command::Command8(0x94, SpiDataMode::Single),
            Address::Address32(0x000000_00, SpiDataMode::Quad),
            4,
            &mut data,
        )
        .unwrap();
        println!("Quad {:x?}", data);
        delay.delay_millis(1500);
    }
}

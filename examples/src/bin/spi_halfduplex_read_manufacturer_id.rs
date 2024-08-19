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
    gpio::Io,
    prelude::*,
    spi::{
        master::{Address, Command, HalfDuplexReadWrite, Spi},
        SpiDataMode,
        SpiMode,
    },
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let System {
        peripherals,
        clocks,
        ..
    } = esp_hal::init(CpuClock::boot_default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let sclk = io.pins.gpio0;
            let miso = io.pins.gpio2;
            let mosi = io.pins.gpio4;
            let sio2 = io.pins.gpio5;
            let sio3 = io.pins.gpio13;
            let cs = io.pins.gpio14;
        } else {
            let sclk = io.pins.gpio0;
            let miso = io.pins.gpio1;
            let mosi = io.pins.gpio2;
            let sio2 = io.pins.gpio3;
            let sio3 = io.pins.gpio4;
            let cs = io.pins.gpio5;
        }
    }

    let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            Some(sio2),
            Some(sio3),
            Some(cs),
        );

    let delay = Delay::new(&clocks);

    loop {
        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.read(
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
        spi.read(
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
        spi.read(
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

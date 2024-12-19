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
    entry,
    spi::{
        master::{Address, Command, Config, Spi},
        DataMode,
        Mode,
    },
    time::RateExtU32,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let sclk = peripherals.GPIO0;
            let miso = peripherals.GPIO2;
            let mosi = peripherals.GPIO4;
            let sio2 = peripherals.GPIO5;
            let sio3 = peripherals.GPIO13;
            let cs = peripherals.GPIO14;
        } else {
            let sclk = peripherals.GPIO0;
            let miso = peripherals.GPIO1;
            let mosi = peripherals.GPIO2;
            let sio2 = peripherals.GPIO3;
            let sio3 = peripherals.GPIO4;
            let cs = peripherals.GPIO5;
        }
    }

    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(100.kHz())
            .with_mode(Mode::Mode0),
    )
    .unwrap()
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
            DataMode::Single,
            Command::Command8(0x90, DataMode::Single),
            Address::Address24(0x000000, DataMode::Single),
            0,
            &mut data,
        )
        .unwrap();
        println!("Single {:x?}", data);
        delay.delay_millis(250);

        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.half_duplex_read(
            DataMode::Dual,
            Command::Command8(0x92, DataMode::Single),
            Address::Address32(0x000000_00, DataMode::Dual),
            0,
            &mut data,
        )
        .unwrap();
        println!("Dual {:x?}", data);
        delay.delay_millis(250);

        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.half_duplex_read(
            DataMode::Quad,
            Command::Command8(0x94, DataMode::Single),
            Address::Address32(0x000000_00, DataMode::Quad),
            4,
            &mut data,
        )
        .unwrap();
        println!("Quad {:x?}", data);
        delay.delay_millis(1500);
    }
}

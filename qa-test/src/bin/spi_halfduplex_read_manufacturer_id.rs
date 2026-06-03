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
//! - SCLK        =>  GPIO12
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

//% CHIP_FILTER: spi_master_driver_supported
//% TAG: flashchip

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    spi::{
        Mode,
        master::{Address, Command, Config, DataMode, Spi},
    },
    time::Rate,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let (sclk, miso, mosi, sio2, sio3, cs) = cfg_select! {
        feature = "esp32" => (
            peripherals.GPIO12,
            peripherals.GPIO2,
            peripherals.GPIO4,
            peripherals.GPIO5,
            peripherals.GPIO13,
            peripherals.GPIO14,
        ),
        _ => (
            peripherals.GPIO0,
            peripherals.GPIO1,
            peripherals.GPIO2,
            peripherals.GPIO3,
            peripherals.GPIO4,
            peripherals.GPIO5,
        ),
    };

    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_sio0(mosi)
    .with_sio1(miso)
    .with_sio2(sio2)
    .with_sio3(sio3)
    .with_cs(cs);

    let delay = Delay::new();

    loop {
        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::_8Bit(0x90, DataMode::SingleTwoDataLines),
            Address::_24Bit(0x000000, DataMode::SingleTwoDataLines),
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
            Command::_8Bit(0x92, DataMode::SingleTwoDataLines),
            Address::_32Bit(0x000000_00, DataMode::Dual),
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
            Command::_8Bit(0x94, DataMode::SingleTwoDataLines),
            Address::_32Bit(0x000000_00, DataMode::Quad),
            4,
            &mut data,
        )
        .unwrap();
        println!("Quad {:x?}", data);
        delay.delay_millis(1500);
    }
}

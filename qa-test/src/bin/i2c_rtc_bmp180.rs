//! Read calibration data from BMP180 sensor
//!
//! This example dumps the calibration data from a BMP180 sensor
//!
//! The following wiring is assumed:
//! - SDA => GPIO3
//! - SCL => GPIO2

//% CHIPS: esp32s3
//% TAG: bmp180

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    i2c::rtc::{Config, I2c, Timing},
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:

    let config = Config::default()
        .with_timing(Timing::standard_mode())
        .with_timeout(Duration::from_micros(100));

    let mut i2c = I2c::new(
        peripherals.RTC_I2C,
        config,
        peripherals.GPIO3,
        peripherals.GPIO2,
    )
    .unwrap();

    loop {
        let mut data = [0u8; 22];
        i2c.read(0x77, 0xaa, &mut data).ok();

        println!("{:02x?}", data);
    }
}

//! Test an SSD1306 display with various read/write data lengths.
//!
//! The following wiring is assumed:
//! - SDA => GPIO4
//! - SCL => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% TAG: ssd1306

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    i2c::master::{Config, I2c},
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:
    let mut i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO4)
        .with_scl(peripherals.GPIO5);

    let mut data = [0u8; 1024];

    println!("Testing writes");
    for limit in 0..data.len() {
        if let Err(e) = i2c.write(0x3c, &data[0..limit]) {
            println!("Error with len {}: {}", limit, e);
        }
        println!("{} bytes ok", limit);
    }

    println!("Testing reads");
    for limit in 0..data.len() {
        if let Err(e) = i2c.read(0x3c, &mut data[0..limit]) {
            println!("Error with len {}: {}", limit, e);
        }
        println!("{} bytes ok", limit);
    }

    println!("Done");

    loop {}
}

//! Read calibration data from BMP180 sensor
//!
//! This example dumps the calibration data from a BMP180 sensor
//!
//! The following wiring is assumed:
//! - SDA => GPIO4
//! - SCL => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{gpio::Io, i2c::I2C, prelude::*};
use esp_println::println;

#[entry]
fn main() -> ! {
    let System {
        peripherals,
        clocks,
        ..
    } = esp_hal::init(CpuClock::boot_default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create a new peripheral object with the described wiring and standard
    // I2C clock speed:
    let mut i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        100.kHz(),
        &clocks,
    );

    loop {
        let mut data = [0u8; 22];
        i2c.write_read(0x77, &[0xaa], &mut data).ok();

        println!("{:02x?}", data);
    }
}

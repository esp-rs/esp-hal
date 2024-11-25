//! Uses `LP_I2C` and reads calibration data from BMP180 sensor.
//!
//! This example dumps the calibration data from a BMP180 sensor, to view them,
//! logic analyzer or oscilloscope is required.
//!
//! The following wiring is assumed:
//! - SDA => GPIO6
//! - SCL => GPIO7

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use embedded_hal::i2c::I2c;
use esp_lp_hal::{i2c::LpI2c, prelude::*};
use panic_halt as _;

#[entry]
fn main(mut i2c: LpI2c) -> ! {
    let _peripherals = esp32c6_lp::Peripherals::take().unwrap();

    loop {
        let mut data = [0u8; 22];
        i2c.write_read(0x77, &[0xaa], &mut data).ok();
    }
}

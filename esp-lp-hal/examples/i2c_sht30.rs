//! Uses `LP_I2C` and reads temperature and humidity from SHT30 sensor.
//!
//! This code runs on the LP core and writes the sensor data to LP memory,
//! which can then be accessed by the HP core.
//!
//! The following wiring is assumed:
//! - SDA => GPIO6
//! - SCL => GPIO7

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use esp_lp_hal::{
    delay::Delay,
    i2c::{Error, LpI2c},
    prelude::*,
};
use panic_halt as _;

// LP SRAM addresses used as a simple shared-memory interface between the LP and HP cores.
// These addresses must match the locations that the HP core expects to read from.
//  - 0x5000_2000: f32 temperature value
//  - 0x5000_2004: f32 humidity value (4 bytes after temperature to hold a second f32)
// Adjust with care: changing these requires updating the corresponding HP-core code and
// ensuring the chosen addresses stay within a valid, reserved LP memory region.
const TEMP_ADDRESS: u32 = 0x5000_2000;
const HUMID_ADDRESS: u32 = 0x5000_2004;

// I2C address of the SHT30 temperature and humidity sensor.
const DEV_ADDR: u8 = 0x44;
// SHT30 command for single-shot measurement, clock stretching disabled, high repeatability.
const CMD_READ_ONESHOT: [u8; 2] = [0x2C, 0x06];

fn read_temp_humid(i2c: &mut LpI2c) -> Result<(f32, f32), Error> {
    let mut buffer = [0u8; 6];
    // Send single-shot measurement command.
    i2c.write(DEV_ADDR, &CMD_READ_ONESHOT)?;
    // Wait for the measurement to complete (up to 15 ms per datasheet).
    Delay.delay_ms(15);
    // Read measurement results.
    i2c.read(DEV_ADDR, &mut buffer)?;
    let temp_raw = u16::from_be_bytes([buffer[0], buffer[1]]);
    let hum_raw = u16::from_be_bytes([buffer[3], buffer[4]]);
    let temperature = -45.0 + (175.0 * (temp_raw as f32) / 65535.0);
    let humidity = 100.0 * (hum_raw as f32) / 65535.0;
    Ok((temperature, humidity))
}

#[entry]
fn main(mut i2c: LpI2c) -> ! {
    let temp_ptr = TEMP_ADDRESS as *mut f32;
    let hum_ptr = HUMID_ADDRESS as *mut f32;
    loop {
        let (temp, humid) = read_temp_humid(&mut i2c).unwrap_or((f32::NAN, f32::NAN));
        unsafe {
            temp_ptr.write_volatile(temp);
            hum_ptr.write_volatile(humid);
        }
        Delay.delay_ms(1000);
    }
}

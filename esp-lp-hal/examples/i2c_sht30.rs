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

const TEMP_ADDRESS: u32 = 0x5000_2000;
const HUMID_ADDRESS: u32 = 0x5000_2004;

const DEV_ADDR: u8 = 0x44;
const CMD_READ_ONESHOT: [u8; 2] = [0x2C, 0x06];

fn read_temp_humid(i2c: &mut LpI2c) -> Result<(f32, f32), Error> {
    let mut buffer = [0u8; 6];
    i2c.write_read(DEV_ADDR, &CMD_READ_ONESHOT, &mut buffer)?;
    let temp_raw = u16::from_be_bytes([buffer[0], buffer[1]]);
    let hum_raw = u16::from_be_bytes([buffer[3], buffer[4]]);
    let temperature = -45.0 + (175.0 * (temp_raw as f32) / 65535.0);
    let humidity = 100.0 * (hum_raw as f32) / 65535.0;
    Ok((temperature, humidity))
}

#[entry]
fn main(mut i2c: LpI2c) -> ! {
    let temp_ptr = TEMP_ADDRESS as *mut f32;
    let humid_ptr = HUMID_ADDRESS as *mut f32;
    loop {
        if let Ok((temp, humid)) = read_temp_humid(&mut i2c) {
            unsafe {
                temp_ptr.write_volatile(temp);
                humid_ptr.write_volatile(humid);
            }
        }
        Delay.delay_ms(1000);
    }
}

//! Read calibration data from ICM42670 sensor
//!
//! This example dumps the accelerometer and gyroscope data from an ICM42670
//! sensor
//!
//! The following wiring is assumed:
//! - SDA => GPIO10
//! - SCL => GPIO8

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    Delay,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create a new peripheral object with the described wiring
    // and standard I2C clock speed
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio10,
        io.pins.gpio8,
        400u32.kHz(),
        &clocks,
    );

    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    use icm42670::accelerometer::Accelerometer;
    let mut imu = icm42670::Icm42670::new(i2c, icm42670::Address::Primary).unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        let gyro_norm = imu.gyro_norm().unwrap();
        let accelerometer = imu.accel_norm().unwrap();
        esp_println::println!("Gyro norm: {gyro_norm:?}; Accel: {accelerometer:?}");

        delay.delay_ms(1000_u32);
    }
}

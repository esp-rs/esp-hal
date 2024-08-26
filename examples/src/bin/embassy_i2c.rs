//! Embassy I2C
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This is an example of running the embassy executor with IC2. It uses an
//! LIS3DH to get accelerometer data.
//!
//! Folowing pins are used:
//! - SDA => GPIO4
//! - SCL => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{gpio::Io, i2c::I2C, prelude::*, timer::timg::TimerGroup};
use lis3dh_async::{Lis3dh, Range, SlaveAddr};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let (peripherals, clocks) = esp_hal::init(Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        400.kHz(),
        &clocks,
    );

    let mut lis3dh = Lis3dh::new_i2c(i2c0, SlaveAddr::Alternate).await.unwrap();
    lis3dh.set_range(Range::G8).await.unwrap();

    loop {
        let norm = lis3dh.accel_norm().await.unwrap();
        esp_println::println!("X: {:+.5}  Y: {:+.5}  Z: {:+.5}", norm.x, norm.y, norm.z);

        Timer::after(Duration::from_millis(100)).await;
    }
}

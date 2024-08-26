//! Embassy "async" vesrion of ead calibration data from BMP180 sensor
//!
//! This example dumps the calibration data from a BMP180 sensor by reading by reading
//! with the direct I2C API and the embedded-hal-async I2C API.
//!
//! Folowing pins are used:
//! - SDA => GPIO4
//! - SCL => GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{gpio::Io, i2c::I2C, prelude::*, timer::timg::TimerGroup};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let (peripherals, clocks) = esp_hal::init(Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut i2c = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio5,
        400.kHz(),
        &clocks,
    );

    loop {
        let mut data = [0u8; 22];
        i2c.write_read(0x77, &[0xaa], &mut data).await.unwrap();
        esp_println::println!("direct:       {:02x?}", data);
        read_data(&mut i2c).await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}

async fn read_data<I2C>(i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut data = [0u8; 22];
    i2c.write_read(0x77, &[0xaa], &mut data).await.unwrap();

    esp_println::println!("embedded_hal: {:02x?}", data);
}

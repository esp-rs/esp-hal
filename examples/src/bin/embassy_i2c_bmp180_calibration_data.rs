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
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

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

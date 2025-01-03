//! This example uses the internal temperature sensor to measure the chip's
//! temperature

//% CHIPS: esp32c6 esp32c3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    entry,
    tsens::{Config, TemperatureSensor},
};
use esp_println::println;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let temperature_sensor = TemperatureSensor::new(peripherals.TSENS, Config::default()).unwrap();
    let delay = Delay::new();

    // Wait for the sensor to stabilize
    delay.delay_micros(200);

    loop {
        let temp = temperature_sensor.get_temperature();
        println!("Temperature: {:.2}°C", temp.to_celsius());
        delay.delay_millis(1_000);
    }
}

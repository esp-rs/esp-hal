//! This example uses the internal temperature sensor to measure the chip's temperature
//!

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, entry, tsens::TemperatureSensor};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let temperature_sensor = TemperatureSensor::new(peripherals.TSENS);
    let delay = Delay::new();
    loop {
        let temp = temperature_sensor.get_celsius();
        println!("Temperature: {:.2}°C", temp);
        delay.delay_millis(1_000);
    }
}

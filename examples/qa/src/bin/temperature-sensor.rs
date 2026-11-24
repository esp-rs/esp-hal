//! This example uses the internal temperature sensor to measure the chip's
//! temperature

//% CHIP_FILTER: temp_sensor_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    tsens::{Config, TemperatureSensor},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut temperature_sensor =
        TemperatureSensor::new(peripherals.TSENS, Config::default()).unwrap();
    let delay = Delay::new();

    loop {
        let sample = temperature_sensor.get_temperature();
        if let Some(celsius) = sample.to_celsius() {
            println!("Temperature: {:.2} °C", celsius);
        } else {
            println!(
                "Uncalibrated temperature: {:.2} °C",
                sample.to_uncalibrated_celsius()
            );
        }
        delay.delay_millis(1_000);
    }
}

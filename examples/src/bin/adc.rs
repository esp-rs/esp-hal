//! Connect a potentiometer to an IO pin and see the read values change when
//! rotating the shaft.
//!
//! Alternatively, you could also connect the IO pin to GND or 3V3 to see the
//! maximum and minimum raw values read.
//!
//! The following wiring is assumed for ESP32:
//! - Analog pin => GPIO32
//! The following wiring is assumed for ESP32S2/S3:
//! - Analog pin => GPIO3
//! The following wiring is assumed for others:
//! - Analog pin => GPIO2

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    gpio::Io,
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let analog_pin = io.pins.gpio32;
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let analog_pin = io.pins.gpio3;
        } else {
            let analog_pin = io.pins.gpio2;
        }
    }

    // Create ADC instances
    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin = adc1_config.enable_pin(analog_pin, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let delay = Delay::new(&clocks);

    loop {
        let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
        println!("ADC reading = {}", pin_value);
        delay.delay_millis(1500);
    }
}

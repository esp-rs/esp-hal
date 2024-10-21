//! Connect an oscilloscope to an IO pin and see the modulation sawtooth wave.
//!
//! Also you may connect low-pass filter to SDM output.
//!
//! The following wiring is assumed for ESP32:
//! - SDM pin => GPIO32
//! The following wiring is assumed for ESP32S2/S3:
//! - SDM pin => GPIO3
//! The following wiring is assumed for others:
//! - SDM pin => GPIO2

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, gpio::Io, prelude::*, sdm::Sdm};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let modulation_pin = io.pins.gpio32;
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let modulation_pin = io.pins.gpio3;
        } else {
            let modulation_pin = io.pins.gpio2;
        }
    }

    let sdm = Sdm::new(peripherals.GPIO_SD);
    let sdm_ch = sdm.channel0.connect(modulation_pin, 100.kHz()).unwrap();

    let delay = Delay::new();

    println!("Sigma-delta modulation is on");

    loop {
        for density in -90..90 {
            sdm_ch.set_pulse_density(density);
            delay.delay_millis(1);
        }
    }
}

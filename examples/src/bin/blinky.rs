//! Blinks an LED
//!
//! The following wiring is assumed:
//! - LED => GPIO0

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output},
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Set GPIO0 as an output, and set its state high initially.

    let mut led = Output::new(peripherals.GPIO0, Level::High);

    let delay = Delay::new();

    loop {
        led.toggle();
        delay.delay_millis(500);
        led.toggle();
        // or using `fugit` duration
        delay.delay(2.secs());
    }
}

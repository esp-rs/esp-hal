//% CHIPS: esp32p4

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::prelude::*;

#[entry]
fn main() -> ! {
    esp_println::println!("Hello world, from ESP32-P4 :)");

    loop {}
}

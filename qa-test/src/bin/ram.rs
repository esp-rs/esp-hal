//! This shows how to use RTC memory.
//!
//! RTC memory is retained during resets and during most sleep modes.
//!
//! Initialized memory is always re-initialized on startup.
//!
//! Persistent memory is not zeroed after resets that preserve RTC ram. See the
//! documentation for `esp-hal-procmacros` for the full list.
//!
//! Zeroed memory is initialized to zero on startup.
//!
//! We can also run code from RTC memory.

//% CHIPS: esp32c61

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::main;
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    println!("Hello World!",);
    loop {}
}

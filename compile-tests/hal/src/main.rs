//! Minimal esp-hal program. Exists so that every chip is covered by at least one
//! compile-test project, including chips no radio project selects.

#![no_std]
#![no_main]

//% CHIP_FILTER: true

use esp_backtrace as _;
use esp_hal::main;
use esp_println as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let _peripherals = esp_hal::init(esp_hal::Config::default());

    loop {}
}

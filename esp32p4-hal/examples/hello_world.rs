#![no_std]
#![no_main]

use esp32p4_hal::prelude::*;
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    esp_println::println!("Hello, world!");
    loop {}
}

//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use esp32c3_hal::{clock::ClockControl, efuse::Efuse, peripherals::Peripherals, prelude::*};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    println!("MAC address {:02x?}", Efuse::get_mac_address());
    println!("Flash Encryption {:?}", Efuse::get_flash_encryption());

    loop {}
}

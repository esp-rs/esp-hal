//! This shows how to read selected information from eFuses.
//!
//! Depending on which chip is being targeted, certain efuses may or may not be
//! available.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{efuse::Efuse, prelude::*};
use esp_println::println;

#[entry]
fn main() -> ! {
    println!("MAC address {:02x?}", Efuse::mac_address());
    println!("Flash Encryption {:?}", Efuse::flash_encryption());

    #[cfg(feature = "esp32")]
    {
        println!("Core Count {}", Efuse::core_count());
        println!("Bluetooth enabled {}", Efuse::is_bluetooth_enabled());
        println!("Chip type {:?}", Efuse::chip_type());
        println!("Max CPU clock {:?}", Efuse::max_cpu_frequency());
    }

    loop {}
}

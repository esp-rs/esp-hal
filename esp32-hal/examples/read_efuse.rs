//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use esp32_hal::{clock::ClockControl, efuse::Efuse, peripherals::Peripherals, prelude::*};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    println!("MAC address {:02x?}", Efuse::get_mac_address());
    println!("Core Count {}", Efuse::get_core_count());
    println!("Bluetooth enabled {}", Efuse::is_bluetooth_enabled());
    println!("Chip type {:?}", Efuse::get_chip_type());
    println!("Max CPU clock {:?}", Efuse::get_max_cpu_frequency());
    println!("Flash Encryption {:?}", Efuse::get_flash_encryption());

    loop {}
}

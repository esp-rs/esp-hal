//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    efuse::Efuse,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    println!("MAC address {:02x?}", Efuse::get_mac_address());
    println!("Core Count {}", Efuse::get_core_count());
    println!("Bluetooth enabled {}", Efuse::is_bluetooth_enabled());
    println!("Chip type {:?}", Efuse::get_chip_type());
    println!("Max CPU clock {:?}", Efuse::get_max_cpu_frequency());
    println!("Flash Encryption {:?}", Efuse::get_flash_encryption());

    loop {}
}

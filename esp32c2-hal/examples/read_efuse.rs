//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use esp32c2_hal::{
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
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    println!("MAC address {:02x?}", Efuse::get_mac_address());
    println!("Flash Encryption {:?}", Efuse::get_flash_encryption());

    loop {}
}

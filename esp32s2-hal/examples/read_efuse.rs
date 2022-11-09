//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    efuse::Efuse,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;
use xtensa_atomic_emulation_trap as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();
    println!("MAC address {:02x?}", Efuse::get_mac_address());
    println!("Flash Encryption {:?}", Efuse::get_flash_encryption());

    loop {}
}

#[xtensa_lx_rt::exception]
fn exception(cause: xtensa_lx_rt::exception::ExceptionCause, frame: xtensa_lx_rt::exception::Context) {
    use esp_println::*;

    println!("\n\nException occured {:?} {:x?}", cause, frame);
    
    let backtrace = esp_backtrace::arch::backtrace();
    for b in backtrace.iter() {
        if let Some(addr) = b {
            println!("0x{:x}", addr)
        }
    }
}
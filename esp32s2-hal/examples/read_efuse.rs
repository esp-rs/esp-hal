//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s2_hal::{
    clock::ClockControl,
    efuse::Efuse,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Serial,
};
use esp_backtrace as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut serial0 = Serial::new(peripherals.UART0);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();
    writeln!(serial0, "MAC address {:02x?}", Efuse::get_mac_address()).unwrap();
    writeln!(
        serial0,
        "Flash Encryption {:?}",
        Efuse::get_flash_encryption()
    )
    .unwrap();

    loop {}
}

//! This shows how to read selected information from eFuses.
//! e.g. the MAC address

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32_hal::{
    clock::ClockControl,
    efuse::Efuse,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    RtcCntl,
    Serial,
};
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut serial0 = Serial::new(peripherals.UART0);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc_cntl.set_wdt_global_enable(false);

    writeln!(serial0, "MAC address {:02x?}", Efuse::get_mac_address()).unwrap();
    writeln!(serial0, "Core Count {}", Efuse::get_core_count()).unwrap();
    writeln!(
        serial0,
        "Bluetooth enabled {}",
        Efuse::is_bluetooth_enabled()
    )
    .unwrap();
    writeln!(serial0, "Chip type {:?}", Efuse::get_chip_type()).unwrap();
    writeln!(serial0, "Max CPU clock {:?}", Efuse::get_max_cpu_fequency()).unwrap();
    writeln!(
        serial0,
        "Flash Encryption {:?}",
        Efuse::get_flash_encryption()
    )
    .unwrap();

    loop {}
}

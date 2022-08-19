//! This shows how to output text via USB Serial/JTAG.
//! You need to connect via the Serial/JTAG interface to see any output.
//! Most dev-kits use a USB-UART-bridge - in that case you won't see any output.
//! This will work with the ESP32-C3-DevKit-RUST-1

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
    UsbSerialJtag,
};
use esp_backtrace as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut delay = Delay::new(&clocks);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    loop {
        writeln!(UsbSerialJtag, "Hello world!").ok();
        delay.delay_ms(500u32);
    }
}

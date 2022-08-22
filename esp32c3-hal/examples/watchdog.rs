//! This demos the watchdog timer.
//! Basically the same as `hello_world` but if you remove the call to
//! `wdt.feed()` the watchdog will reset the system.

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Serial,
};
use esp_backtrace as _;
use nb::block;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut serial0 = Serial::new(peripherals.UART0);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.start(2u64.secs());
    wdt1.disable();

    timer0.start(1u64.secs());

    loop {
        wdt0.feed();
        writeln!(serial0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}

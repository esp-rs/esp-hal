//! This shows how to write text to uart0.
//! You can see the output with `espflash` if you provide the `--monitor` option

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use nb::block;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut uart = peripherals.UART0;
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;
    
    let mut uart0 = Uart::new(&mut uart);
    writeln!(uart0, "Test 1").unwrap();
    drop(uart0); // this ends the mutable borrow of uart, its now available in the peripheral struct again
    let mut uart0 = Uart::new(uart); // construct via move this time
    writeln!(uart0, "Test 2").unwrap();

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    timer0.start(1u64.secs());

    loop {
        writeln!(uart0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}

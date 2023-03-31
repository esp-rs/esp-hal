//! This shows how to write text to serial0.
//! You can see the output with `espflash` if you provide the `--monitor` option

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use nb::block;

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
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut serial0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    timer0.start(1u64.secs());

    loop {
        writeln!(serial0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}

//! This shows how to write text to serial0.
//! You can see the output with `espflash` if you provide the `--monitor` option

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s2_hal::{
    clock::ClockControl,
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Serial,
};
use esp_backtrace as _;
use xtensa_atomic_emulation_trap as _;
use nb::block;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut serial0 = Serial::new(peripherals.UART0);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    timer0.start(1u64.secs());

    loop {
        writeln!(serial0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
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

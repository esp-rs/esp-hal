//! This shows how to write text to UART0.
//!
//! You can see the output with `espflash` if you provide the `--monitor`
//! option.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::fmt::Write;

use esp_backtrace as _;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Uart};
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timg0.timer0;
    timer0.start(1u64.secs());

    let mut uart0 = Uart::new(peripherals.UART0, &clocks);

    loop {
        writeln!(uart0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}

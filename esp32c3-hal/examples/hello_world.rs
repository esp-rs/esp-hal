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
    Uart,
};
use esp_backtrace as _;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut uart0 = Uart::new(peripherals.UART0, &clocks);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;

    timer0.start(1u64.secs());

    loop {
        writeln!(uart0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}

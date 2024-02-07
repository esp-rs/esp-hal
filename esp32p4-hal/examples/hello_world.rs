#![no_std]
#![no_main]

use core::fmt::Write;

use esp32p4_hal::{
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    prelude::*,
    Uart,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock90MHz).freeze();

    let mut uart0 = Uart::new(peripherals.UART0, &clocks);
    writeln!(uart0, "Hello, world!").unwrap();

    loop {}
}

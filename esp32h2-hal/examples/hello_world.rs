#![no_std]
#![no_main]

use esp32h2_hal::{
    clock::{ClockControl, CpuClock},
    entry,
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.PCR.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock96MHz).freeze();

    println!("Hello, world!");
    loop {}
}

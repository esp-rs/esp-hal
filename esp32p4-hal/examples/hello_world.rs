#![no_std]
#![no_main]

use esp32p4_hal::{
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    prelude::*,
    system::SystemExt,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock90MHz).freeze();

    esp_println::println!("Hello, world!");
    loop {}
}

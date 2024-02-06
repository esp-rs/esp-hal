#![no_std]
#![no_main]

use esp32p4_hal::{
    clock::ClockControl, peripherals::Peripherals, prelude::*, system::SystemExt, Delay,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut delay = Delay::new(&clocks);

    loop {
        esp_println::println!("Hello, world!");
        delay.delay_ms(1000 as u32);
    }
}

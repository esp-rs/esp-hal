//! This demos the watchdog timer.
//!
//! Basically the same as `hello_world` but if you remove the call to
//! `wdt.feed()` the watchdog will reset the system.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    let mut wdt0 = timg0.wdt;
    wdt0.enable();
    wdt0.set_timeout(2u64.secs());

    loop {
        wdt0.feed();
        println!("Hello world!");
        delay.delay(1.secs());
    }
}

//! embassy systimer delay
//!
//! This is an example of using the `DelayNs` trait implementation

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-time-timg0 embassy-generic-timers async

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embedded_hal_async::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{systimer::SystemTimer, timg::TimerGroup},
};

#[main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    let mut alarm0 = SystemTimer::new_async(peripherals.SYSTIMER)
        .alarm0
        .into_periodic();

    loop {
        esp_println::println!("Bing!");
        alarm0.delay_ms(1000).await;
    }
}

//! embassy hello world systimer
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently using the systimer time driver.
//!
//! It's not supported on ESP32, on ESP32-S2 the frequency of the systimer is different (so it's left out here)

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s3
//% FEATURES: embassy embassy-time-systick-16mhz embassy-executor-thread embassy-generic-timers

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::systimer::SystemTimer,
};

#[embassy_executor::task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let systimer = SystemTimer::new_async(peripherals.SYSTIMER);
    embassy::init(&clocks, systimer);

    spawner.spawn(run()).ok();

    loop {
        esp_println::println!("Bing!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

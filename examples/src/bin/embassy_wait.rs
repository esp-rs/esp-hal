//! embassy wait
//!
//! This is an example of asynchronously `Wait`ing for a pin state to change.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-time-timg0 embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
};

#[main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let mut input = Input::new(io.pins.gpio0, Pull::Down);
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let mut input = Input::new(io.pins.gpio9, Pull::Down);

    loop {
        esp_println::println!("Waiting...");
        input.wait_for_rising_edge().await;
        esp_println::println!("Ping!");
        Timer::after(Duration::from_millis(100)).await;
    }
}

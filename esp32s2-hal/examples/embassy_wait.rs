//! embassy wait
//!
//! This is an example of asynchronously `Wait`ing for a pin state to change.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use esp32s2_hal::{
    clock::ClockControl,
    embassy::{self},
    peripherals::Peripherals,
    prelude::*,
    IO,
};
use esp_backtrace as _;

#[main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32s2_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // GPIO 0 as input
    let mut input = io.pins.gpio0.into_pull_down_input();

    // Async requires the GPIO interrupt to wake futures
    esp32s2_hal::interrupt::enable(
        esp32s2_hal::peripherals::Interrupt::GPIO,
        esp32s2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    loop {
        esp_println::println!("Waiting...");
        input.wait_for_rising_edge().await.unwrap();
        esp_println::println!("Ping!");
        Timer::after(Duration::from_millis(100)).await;
    }
}

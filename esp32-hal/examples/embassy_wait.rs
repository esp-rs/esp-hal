//! embassy wait
//!
//! This is an example of asynchronously `Wait`ing for a pin state to change.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use esp32_hal::{
    clock::ClockControl,
    embassy::{self, executor::Executor},
    gpio::{Gpio0, Input, PullDown},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    IO,
};
use esp_backtrace as _;
use static_cell::make_static;

#[embassy_executor::task]
async fn ping(mut pin: Gpio0<Input<PullDown>>) {
    loop {
        esp_println::println!("Waiting...");
        pin.wait_for_rising_edge().await.unwrap();
        esp_println::println!("Ping!");
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // GPIO 0 as input
    let input = io.pins.gpio0.into_pull_down_input();

    // Async requires the GPIO interrupt to wake futures
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::GPIO,
        esp32_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(ping(input)).ok();
    });
}

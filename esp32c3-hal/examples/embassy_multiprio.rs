//! This example shows how to use the interrupt executors to prioritize some
//! tasks over others. The low priority task will not be able to run its async
//! task while the blocking task is running, but the high priority task will be
//! able to blink the LED regardless.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp32c3_hal::{
    clock::ClockControl,
    embassy::{
        self,
        executor::{FromCpu1, InterruptExecutor},
    },
    interrupt::Priority,
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;
use esp_println::println;

static INT_EXECUTOR_0: InterruptExecutor<FromCpu1> = InterruptExecutor::new();

#[interrupt]
fn FROM_CPU_INTR1() {
    unsafe { INT_EXECUTOR_0.on_interrupt() }
}

/// Periodically print something.
#[embassy_executor::task]
async fn high_prio() {
    println!("Starting high_prio()");
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        println!("High priority ticks");
        ticker.next().await;
    }
}

/// Simulates some blocking (badly behaving) task.
#[embassy_executor::task]
async fn low_prio_blocking() {
    println!("Starting low-priority task that isn't actually async");
    loop {
        println!("Doing some long and complicated calculation");
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {}
        println!("Calculation finished");
    }
}

#[main]
async fn main(low_prio_spawner: Spawner) {
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32c3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32c3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let spawner = INT_EXECUTOR_0.start(Priority::Priority2);
    spawner.must_spawn(high_prio());

    println!("spawning");
    low_prio_spawner.must_spawn(low_prio_blocking());
}

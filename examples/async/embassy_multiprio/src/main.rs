//! This example shows how to use the interrupt executors to prioritize some
//! tasks over others.
//!
//! The example creates three tasks:
//!  - A low priority task that is not actually async, but simulates some blocking work. This task
//!    will run for 5 seconds, then sleep for 5 seconds.
//!  - A low priority task that is actually async, but will not be able to run while the blocking
//!    task is running.
//!  - A high priority task that prints something every second. The example demonstrates that this
//!    task will continue to run even while the low priority blocking task is running.

// The thread-executor is created by the `#[esp_rtos::main]` macro and is used to spawn
// `low_prio_async` and `low_prio_blocking`. The interrupt-executor is created in `main` and is used
// to spawn `high_prio`.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    interrupt::{Priority, software::SoftwareInterruptControl},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_rtos::embassy::InterruptExecutor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

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
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// A well-behaved, but starved async task.
#[embassy_executor::task]
async fn low_prio_async() {
    println!(
        "Starting low-priority task that will not be able to run while the blocking task is running"
    );
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        println!("Low priority ticks");
        ticker.next().await;
    }
}

#[esp_rtos::main]
async fn main(low_prio_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    println!("Init!");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    static EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    let executor = InterruptExecutor::new(sw_int.software_interrupt2);
    let executor = EXECUTOR.init(executor);

    let spawner = executor.start(Priority::Priority3);
    spawner.must_spawn(high_prio());

    println!("Spawning low-priority tasks");
    low_prio_spawner.must_spawn(low_prio_async());
    low_prio_spawner.must_spawn(low_prio_blocking());
}

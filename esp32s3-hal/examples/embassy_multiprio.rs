//! This example shows how to use the interrupt executors to prioritize some
//! tasks over others. The low priority task will not be able to run its async
//! task while the blocking task is running, but the high priority task will be
//! able to blink the LED regardless.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_time::{Duration, Instant, Ticker};
use esp32s3_hal::{
    clock::ClockControl,
    embassy::{
        self,
        executor::{FromCpu1, FromCpu2, InterruptExecutor},
    },
    gpio::{GpioPin, Output, PushPull, IO},
    interrupt::Priority,
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;
use esp_hal_common::get_core;
use esp_println::println;
use static_cell::make_static;

static INT_EXECUTOR_0: InterruptExecutor<FromCpu1> = InterruptExecutor::new();
static INT_EXECUTOR_1: InterruptExecutor<FromCpu2> = InterruptExecutor::new();

#[interrupt]
fn FROM_CPU_INTR1() {
    unsafe { INT_EXECUTOR_0.on_interrupt() }
}

#[interrupt]
fn FROM_CPU_INTR2() {
    unsafe { INT_EXECUTOR_1.on_interrupt() }
}

/// Periodically turns the LED on and off.
#[embassy_executor::task]
async fn high_prio(led: &'static mut GpioPin<Output<PushPull>, 0>) {
    println!("Starting high_prio() on core {}", get_core() as usize);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        esp_println::println!("LED on");
        led.set_low().unwrap();
        ticker.next().await;

        esp_println::println!("LED off");
        led.set_high().unwrap();
        ticker.next().await;
    }
}

/// Simulates some blocking (badly behaving) task.
#[embassy_executor::task]
async fn low_prio_blocking() {
    println!(
        "Starting low_prio_blocking() on core {}",
        get_core() as usize
    );
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(10) {}
    esp_println::println!("Low prio task finished");
}

/// Simulates a well-behaved async task that prints to the serial output.
#[embassy_executor::task]
async fn low_prio_async() {
    println!("Starting low_prio_async() on core {}", get_core() as usize);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        // Note that when the blocking task finishes, the ticker will fire multiple
        // times.
        println!("Tick from low priority async task");
        ticker.next().await;
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Set GPIO2 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32s3_hal::timer::TimerGroup::new(
            peripherals.TIMG0,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        embassy::init(&clocks, timer_group0.timer0);
    }

    let led = make_static!(io.pins.gpio0.into_push_pull_output());

    let spawner = INT_EXECUTOR_0.start(Priority::Priority2);
    spawner.spawn(high_prio(led)).ok();

    let spawner = INT_EXECUTOR_1.start(Priority::Priority1);
    spawner.spawn(low_prio_async()).ok();
    spawner.spawn(low_prio_blocking()).ok();

    // Just loop to show that the main thread does not need to poll the executor.
    loop {}
}

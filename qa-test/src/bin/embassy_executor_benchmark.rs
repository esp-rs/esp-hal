//! Embassy executor benchmark, used to try out optimization ideas.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
// FEATURES:
// FEATURES: esp-hal-embassy/single-queue
//% FEATURES: esp-hal-embassy/generic-queue

#![no_std]
#![no_main]

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

use embassy_executor::{raw::TaskStorage, Spawner};
use esp_backtrace as _;
use esp_hal::{
    clock::{Clock, CpuClock},
    handler,
    time::Duration,
    timer::{systimer::SystemTimer, OneShotTimer},
};
use esp_println::println;

static mut COUNTER: u32 = 0;
static mut T2_COUNTER: u32 = 0;
static mut T3_COUNTER: u32 = 0;

const CLOCK: CpuClock = CpuClock::max();
const TEST_MILLIS: u64 = 500;

#[handler]
fn timer_handler() {
    let c = unsafe { COUNTER } as u64;
    let cpu_clock = CLOCK.hz() as u64;
    let timer_ticks_per_second = SystemTimer::ticks_per_second();
    let cpu_cycles_per_timer_ticks = cpu_clock / timer_ticks_per_second;
    println!("task2 count={}", unsafe { T2_COUNTER });
    println!("task3 count={}", unsafe { T3_COUNTER });
    println!(
        "Test OK, count={}, cycles={}/100",
        c,
        (100 * timer_ticks_per_second * cpu_cycles_per_timer_ticks * TEST_MILLIS / 1000) / c
    );
    loop {}
}

struct Task1 {}
impl Future for Task1 {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        unsafe { COUNTER += 1 };
        cx.waker().wake_by_ref();
        Poll::Pending
    }
}

static TASK1: TaskStorage<Task1> = TaskStorage::new();

#[embassy_executor::task]
async fn task2() {
    loop {
        unsafe { T2_COUNTER += 1 };
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn task3() {
    loop {
        unsafe { T3_COUNTER += 1 };
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CLOCK);
    let peripherals = esp_hal::init(config);
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    println!("Embassy initialized!");

    spawner.spawn(TASK1.spawn(|| Task1 {})).unwrap();
    spawner.spawn(task2()).unwrap();
    spawner.spawn(task3()).unwrap();

    println!("Starting test");

    let mut timer = OneShotTimer::new(systimer.alarm1);
    timer.set_interrupt_handler(timer_handler);
    timer.enable_interrupt(true);
    timer.schedule(Duration::millis(TEST_MILLIS)).unwrap();
}

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
    prelude::*,
    time::Duration,
    timer::{
        systimer::{SystemTimer, Target},
        OneShotTimer,
    },
};
use log::info;

static mut COUNTER: u32 = 0;

const CLOCK: CpuClock = CpuClock::max();
const TEST_MILLIS: u64 = 50;

#[handler]
fn timer_handler() {
    let c = unsafe { COUNTER } as u64;
    let cpu_clock = CLOCK.hz() as u64;
    let timer_ticks_per_second = SystemTimer::ticks_per_second();
    let cpu_cycles_per_timer_ticks = cpu_clock / timer_ticks_per_second;
    info!(
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

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CLOCK;
        config
    });

    esp_println::logger::init_logger_from_env();

    let systimer =
        esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(systimer.alarm0);
    info!("Embassy initialized!");

    spawner.spawn(TASK1.spawn(|| Task1 {})).unwrap();

    info!("Starting test");

    let mut timer = OneShotTimer::new(systimer.alarm1);
    timer.set_interrupt_handler(timer_handler);
    timer.enable_interrupt(true);
    timer.schedule(Duration::millis(TEST_MILLIS)).unwrap();
}

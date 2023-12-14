//! This example shows how to spawn async tasks on the second core.
//! The second core runs a simple LED blinking task, that is controlled by a
//! signal set by the task running on the other core.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use esp32s3_hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    embassy::{self, executor::Executor},
    get_core,
    gpio::{GpioPin, Output, PushPull, IO},
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;
use esp_println::println;
use static_cell::make_static;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

/// Waits for a message that contains a duration, then flashes a led for that
/// duration of time.
#[embassy_executor::task]
async fn control_led(
    mut led: GpioPin<Output<PushPull>, 0>,
    control: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    println!("Starting control_led() on core {}", get_core() as usize);
    loop {
        if control.wait().await {
            esp_println::println!("LED on");
            led.set_low().unwrap();
        } else {
            esp_println::println!("LED off");
            led.set_high().unwrap();
        }
    }
}

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
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
        let timer_group0 = esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let mut cpu_control = CpuControl::new(system.cpu_control);

    let led_ctrl_signal = &*make_static!(Signal::new());

    let led = io.pins.gpio0.into_push_pull_output();
    let cpu1_fnctn = move || {
        let executor = make_static!(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(control_led(led, led_ctrl_signal)).ok();
        });
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fnctn)
        .unwrap();

    // Sends periodic messages to control_led, enabling or disabling it.
    println!(
        "Starting enable_disable_led() on core {}",
        get_core() as usize
    );
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        esp_println::println!("Sending LED on");
        led_ctrl_signal.signal(true);
        ticker.next().await;

        esp_println::println!("Sending LED off");
        led_ctrl_signal.signal(false);
        ticker.next().await;
    }
}

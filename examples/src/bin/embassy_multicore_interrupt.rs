//! This example shows how to use the interrupt executors on either core.
//!
//! The second core runs a simple LED blinking task, that is controlled by a
//! signal set by the task running on the other core.

//% CHIPS: esp32 esp32s3
//% FEATURES: embassy embassy-time-timg0 embassy-generic-timers

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    embassy::{self, executor::InterruptExecutor},
    get_core,
    gpio::{GpioPin, Io, Output, PushPull},
    interrupt::Priority,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
};
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
            led.set_low();
        } else {
            esp_println::println!("LED off");
            led.set_high();
        }
    }
}

/// Sends periodic messages to control_led, enabling or disabling it.
#[embassy_executor::task]
async fn enable_disable_led(control: &'static Signal<CriticalSectionRawMutex, bool>) {
    println!(
        "Starting enable_disable_led() on core {}",
        get_core() as usize
    );
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        esp_println::println!("Sending LED on");
        control.signal(true);
        ticker.next().await;

        esp_println::println!("Sending LED off");
        control.signal(false);
        ticker.next().await;
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    let led_ctrl_signal = &*make_static!(Signal::new());

    let led = io.pins.gpio0.into_push_pull_output();

    let executor_core1 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt1);
    let executor_core1 = make_static!(executor_core1);

    let cpu1_fnctn = move || {
        let spawner = executor_core1.start(Priority::Priority1);

        spawner.spawn(control_led(led, led_ctrl_signal)).ok();

        // Just loop to show that the main thread does not need to poll the executor.
        loop {}
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, cpu1_fnctn)
        .unwrap();

    let executor_core0 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt0);
    let executor_core0 = make_static!(executor_core0);

    let spawner = executor_core0.start(Priority::Priority1);
    spawner.spawn(enable_disable_led(led_ctrl_signal)).ok();

    // Just loop to show that the main thread does not need to poll the executor.
    loop {}
}

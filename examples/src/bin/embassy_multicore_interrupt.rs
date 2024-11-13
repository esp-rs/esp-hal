//! This example shows how to use the interrupt executors on either core.
//!
//! The second core runs a simple LED blinking task, that is controlled by a
//! signal set by the task running on the other core.
//!
//! The following wiring is assumed:
//! - LED => GPIO0

//% CHIPS: esp32 esp32s3
//% FEATURES: embassy embassy-generic-timers

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    core,
    cpu_control::{CpuControl, Stack},
    gpio::{Level, Output},
    interrupt::{software::SoftwareInterruptControl, Priority},
    prelude::*,
    timer::{timg::TimerGroup, AnyTimer},
};
use esp_hal_embassy::InterruptExecutor;
use esp_println::println;
use static_cell::StaticCell;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

/// Waits for a message that contains a duration, then flashes a led for that
/// duration of time.
#[embassy_executor::task]
async fn control_led(
    mut led: Output<'static>,
    control: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    println!("Starting control_led() on core {}", core() as usize);
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
    println!("Starting enable_disable_led() on core {}", core() as usize);
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
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();
    esp_hal_embassy::init([timer0, timer1]);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    let led_ctrl_signal = &*LED_CTRL.init(Signal::new());

    let led = Output::new(peripherals.GPIO0, Level::Low);

    static EXECUTOR_CORE_1: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    let executor_core1 = InterruptExecutor::new(sw_ints.software_interrupt1);
    let executor_core1 = EXECUTOR_CORE_1.init(executor_core1);

    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            let spawner = executor_core1.start(Priority::Priority1);

            spawner.spawn(control_led(led, led_ctrl_signal)).ok();

            // Just loop to show that the main thread does not need to poll the executor.
            loop {}
        })
        .unwrap();

    static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<0>> = StaticCell::new();
    let executor_core0 = InterruptExecutor::new(sw_ints.software_interrupt0);
    let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

    let spawner = executor_core0.start(Priority::Priority1);
    spawner.spawn(enable_disable_led(led_ctrl_signal)).ok();

    // Just loop to show that the main thread does not need to poll the executor.
    loop {}
}

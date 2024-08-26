//! This example shows how to spawn async tasks on the second core.
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

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    cpu_control::{CpuControl, Stack},
    get_core,
    gpio::{AnyOutput, Io, Level},
    timer::{timg::TimerGroup, ErasedTimer},
};
use esp_hal_embassy::Executor;
use esp_println::println;
use static_cell::StaticCell;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

/// Waits for a message that contains a duration, then flashes a led for that
/// duration of time.
#[embassy_executor::task]
async fn control_led(
    mut led: AnyOutput<'static>,
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

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let (peripherals, clocks) = esp_hal::init(CpuClock::boot_default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timer1: ErasedTimer = timg0.timer1.into();
    esp_hal_embassy::init(&clocks, [timer0, timer1]);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    let led_ctrl_signal = &*LED_CTRL.init(Signal::new());

    let led = AnyOutput::new(io.pins.gpio0, Level::Low);

    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                spawner.spawn(control_led(led, led_ctrl_signal)).ok();
            });
        })
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

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
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    get_core,
    gpio::{AnyOutput, Io, Level},
    interrupt::Priority,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};
use esp_hal_embassy::InterruptExecutor;
use esp_println::println;
use static_cell::StaticCell;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

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

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timer1 = OneShotTimer::new(timg0.timer1.into());
    let timers = [timer0, timer1];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 2], timers);
    esp_hal_embassy::init(&clocks, timers);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    let led_ctrl_signal = &*LED_CTRL.init(Signal::new());

    let led = AnyOutput::new(io.pins.gpio0, Level::Low);

    static EXECUTOR_CORE_1: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    let executor_core1 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt1);
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
    let executor_core0 =
        InterruptExecutor::new(system.software_interrupt_control.software_interrupt0);
    let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

    let spawner = executor_core0.start(Priority::Priority1);
    spawner.spawn(enable_disable_led(led_ctrl_signal)).ok();

    // Just loop to show that the main thread does not need to poll the executor.
    loop {}
}

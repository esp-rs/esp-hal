//! This example shows how to use the interrupt executors on either core.
//!
//! The second core runs a simple LED blinking task, that is controlled by a
//! signal set by the task running on the other core.
//!
//! The interrupt executor works without the esp_rtos scheduler, so this example uses the esp-hal
//! CpuControl API to start the second core.
//!
//! The following wiring is assumed:
//! - LED => GPIO0

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    interrupt::{Priority, software::SoftwareInterruptControl},
    main,
    system::{Cpu, CpuControl, Stack},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_rtos::embassy::InterruptExecutor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

/// Waits for a message that contains a duration, then flashes a led for that
/// duration of time.
#[embassy_executor::task]
async fn control_led(
    mut led: Output<'static>,
    control: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    println!("Starting control_led() on core {}", Cpu::current() as usize);
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
        Cpu::current() as usize
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

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    let led_ctrl_signal = &*LED_CTRL.init(Signal::new());

    let led = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());

    static EXECUTOR_CORE_1: StaticCell<InterruptExecutor<1>> = StaticCell::new();
    let executor_core1 = InterruptExecutor::new(sw_int.software_interrupt1);
    let executor_core1 = EXECUTOR_CORE_1.init(executor_core1);

    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());

    let _guard = cpu_control
        .start_app_core(app_core_stack, move || {
            let spawner = executor_core1.start(Priority::Priority1);

            spawner.spawn(control_led(led, led_ctrl_signal)).ok();

            // Just loop to show that the main thread does not need to poll the executor.
            loop {}
        })
        .unwrap();

    static EXECUTOR_CORE_0: StaticCell<InterruptExecutor<2>> = StaticCell::new();
    let executor_core0 = InterruptExecutor::new(sw_int.software_interrupt2);
    let executor_core0 = EXECUTOR_CORE_0.init(executor_core0);

    let spawner = executor_core0.start(Priority::Priority1);
    spawner.spawn(enable_disable_led(led_ctrl_signal)).ok();

    // Just loop to show that the main thread does not need to poll the executor.
    loop {}
}

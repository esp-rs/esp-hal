//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn run() {
    loop {
        esp_println::println!("Hello world from embassy!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::println!("Init!");

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    spawner.spawn(run()).ok();

    loop {
        esp_println::println!("Bing!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn run() {
    loop {
        log::info!("Hello world from embassy!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32" => peripherals.GPIO1,
        feature = "esp32c2" => peripherals.GPIO20,
        feature = "esp32c3" => peripherals.GPIO21,
        any(feature = "esp32c5", feature = "esp32c61") => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        any(feature = "esp32h2", feature = "esp32h4") => peripherals.GPIO24,
        feature = "esp32p4" => peripherals.GPIO37,
        any(feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO43,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    log::info!("Init!");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    spawner.spawn(run().unwrap());

    loop {
        log::info!("Bing!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

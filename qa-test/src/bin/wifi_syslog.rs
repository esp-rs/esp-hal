//! Check that the print-logs-from-driver feature works and doesn't crash
//! when wifi driver blobs emit log messages.

//% CHIPS: esp32 esp32s2 esp32s3
//% FEATURES: esp-radio esp-radio/wifi esp-radio/unstable esp-hal/unstable
//% FEATURES: esp-radio/print-logs-from-driver

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{Config, ControllerConfig, sta::StationConfig};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
    );

    println!("Creating wifi controller with print-logs-from-driver enabled");

    let mut controller = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    println!("Connecting to WiFi...");
    controller.connect_async().await.unwrap();
    println!("Connected! Driver syslog output should be visible above.");

    // Disconnect to exercise teardown log paths too
    controller.disconnect_async().await.unwrap();
    println!("Disconnected.");

    println!("Done");
}

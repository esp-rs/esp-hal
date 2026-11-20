//! Check that the print-logs-from-driver feature works and doesn't crash
//! when wifi driver blobs emit log messages.

//% CHIP_FILTER: xtensa && wifi_driver_supported
//% FEATURES: esp-radio esp-radio/wifi esp-radio/unstable esp-hal/unstable
//% FEATURES: esp-radio/print-logs-from-driver

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, timer::timg::TimerGroup};
use esp_radio::wifi::{AuthenticationMethodConfig, Config, ControllerConfig, sta::StationConfig};
use qa_test::DRAM2_UNINIT_SIZE;

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_log::init_uart!(peripherals);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: DRAM2_UNINIT_SIZE);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    log::info!("Creating wifi controller with print-logs-from-driver enabled");

    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    log::info!("Connecting to WiFi...");
    controller.connect_async().await.unwrap();
    log::info!("Connected! Driver syslog output should be visible above.");

    // Disconnect to exercise teardown log paths too
    controller.disconnect_async().await.unwrap();
    log::info!("Disconnected.");

    log::info!("Done");
}

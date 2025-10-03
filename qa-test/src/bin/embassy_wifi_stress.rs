//! This example stress-tests esp-radio and esp-rtos's ability to deal with repeatedly
//! disconnecting and reconnecting to networks.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3
//% FEATURES: esp-radio esp-radio/wifi

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{boxed::Box, string::ToString};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, WithTimeout};
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    Config as EspConfig,
    clock::CpuClock,
    init as initialize_esp_hal,
    ram,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{ClientConfig, ScanConfig, WifiConfig, WifiMode};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let mut peripherals = initialize_esp_hal(EspConfig::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(size: 32 * 1024);
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let esp_wifi_ctrl = Box::leak(Box::new(esp_radio::init().unwrap()));
    let (mut controller, _interfaces) = esp_radio::wifi::new(
        esp_wifi_ctrl,
        peripherals.WIFI.reborrow(),
        WifiConfig::default(),
    )
    .unwrap();

    let mut i = 0;
    loop {
        println!("{}", esp_alloc::HEAP.stats());

        i += 1;
        println!(
            "==================================CONNECTION ATTEMPT=================================="
        );
        println!(
            "===========================================#{}=========================================",
            i
        );
        if controller.is_started().unwrap_or(false) {
            // Timer::after(Duration::from_secs(10)).await; // These sleeps delay reproducing the
            // bug, but do not prevent it
            controller.stop_async().await.unwrap();
            // Timer::after(Duration::from_secs(10)).await;
        }

        controller.set_mode(WifiMode::Sta).unwrap();
        println!("Wifi stack setup (STA)");
        controller.start_async().await.unwrap();
        println!("Connecting to WiFi SSID: {}", SSID);
        let scan_config = ScanConfig::default().with_ssid(&SSID).with_scan_type(
            esp_radio::wifi::ScanTypeConfig::Active {
                min: core::time::Duration::from_millis(5),
                max: core::time::Duration::from_millis(20),
            },
        );
        println!("Scanning for WiFi networks");
        let aps = controller
            .scan_with_config_async(scan_config)
            .with_timeout(Duration::from_secs(5))
            .await
            .unwrap();
        if aps.is_err() {
            println!("Failed to scan wifi networks, timeout hit!");
            Timer::after(Duration::from_secs(2)).await;
            continue;
        }

        let mut aps = aps.unwrap();
        println!("Found {} access points", aps.len());
        if aps.is_empty() {
            println!("No access points found.");
            Timer::after(Duration::from_secs(2)).await;
            continue;
        }
        aps.sort_by(|x, y| y.signal_strength.cmp(&x.signal_strength));
        let best_one = aps.first().unwrap();
        println!("Best AP found: {:?}", best_one);
        println!("Connecting to WiFi SSID: {}", SSID);

        let client_config = esp_radio::wifi::Config::Client(
            ClientConfig::default()
                .with_ssid(best_one.ssid.clone())
                .with_bssid(best_one.bssid)
                .with_auth_method(best_one.auth_method.unwrap())
                .with_password(PASSWORD.to_string())
                .with_channel(best_one.channel),
        );
        controller.set_config(&client_config).unwrap();

        let err = controller
            .connect_async()
            .with_timeout(Duration::from_secs(10))
            .await;

        if err.is_err() {
            println!("Failed to connect to Wifi, timeout hit!");
            Timer::after(Duration::from_secs(2)).await;
            continue;
        }
        let err = err.unwrap();
        if err.is_err() {
            println!("Failed to connect to WiFi: {:?}", err);
            Timer::after(Duration::from_secs(2)).await;
        } else {
            println!("Connect OK");
        }
    }
}

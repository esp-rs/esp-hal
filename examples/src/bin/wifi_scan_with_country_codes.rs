//! WiFi scanning example with country code display
//!
//! This example demonstrates how to scan for WiFi access points and display
//! their country codes, which are now available directly from ESP-IDF.

//% FEATURES: esp-wifi esp-wifi/wifi esp-hal/unstable
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{collections::BTreeSet, string::String, vec::Vec};

use esp_backtrace as _;
use esp_hal::{clock::CpuClock, main, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::{init, wifi::WifiMode};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = init(timg0.timer0, Rng::new(peripherals.RNG)).unwrap();

    let (mut controller, _interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    controller.set_mode(WifiMode::Sta).unwrap();
    controller.start().unwrap();

    println!("Starting WiFi scan with country code display...");

    // Perform WiFi scan
    let scan_results = controller.scan_n(20).unwrap();

    println!("Found {} access points:\n", scan_results.len());
    println!(
        "{:<25} | {:<8} | {:<5} | {:<8}",
        "SSID", "Channel", "RSSI", "Country"
    );
    println!("{:-<50}", "");

    // Display scan results with country codes
    for ap in &scan_results {
        let country_display = ap.country_code.as_deref().unwrap_or("Unknown");
        println!(
            "{:<25} | {:<8} | {:<5} | {:<8}",
            if ap.ssid.len() > 24 {
                &ap.ssid[..24]
            } else {
                &ap.ssid
            },
            ap.channel,
            ap.signal_strength,
            country_display
        );
    }

    // Show unique country codes found
    let unique_countries: Vec<String> = {
        let mut countries: BTreeSet<String> = BTreeSet::new();
        for ap in &scan_results {
            if let Some(ref country) = ap.country_code {
                countries.insert(country.clone());
            }
        }
        countries.into_iter().collect()
    };

    println!("\nUnique country codes detected: {:?}", unique_countries);
    println!(
        "Total APs with country info: {}",
        scan_results
            .iter()
            .filter(|ap| ap.country_code.is_some())
            .count()
    );

    loop {
        let mut delay = esp_hal::delay::Delay::new();
        delay.delay_millis(1000u32);
    }
}

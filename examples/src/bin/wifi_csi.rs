//! DHCP Example
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! This gets an ip address via DHCP then performs an HTTP get request to some "random" server

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/esp-now
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::{
    init,
    wifi::{
        new_with_mode,
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        CsiConfiguration,
        WifiError,
        WifiStaDevice,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let delay = Delay::new();

    let init = init(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut wifi = peripherals.WIFI;
    // ESP-NOW
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();
    println!("esp-now version {}", esp_now.get_version().unwrap());
    esp_now.set_rate(esp_wifi::esp_now::WifiPhyRate::RateMcs0Sgi);
    esp_now.set_channel(11).unwrap();

    delay.delay(4.secs());
    let mut csi = CsiConfiguration::default();
    csi.set_config();
    // csi.set_rx_cb();
    println!("Waiting for CSI data...");
    csi.set_receive_cb(|data| {
        let rx_ctrl = data.rx_ctrl;
        println!("CSI data: {:?}", rx_ctrl.rssi());
    });

    csi.set_csi(true);

    loop {
        delay.delay(4.secs());
    }
}

//! WiFi sniffer example
//!
//! Sniffs for beacon frames.

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/sniffer
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{
    collections::btree_set::BTreeSet,
    string::{String, ToString},
};
use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    prelude::*,
    rng::Rng,
    timer::{timg::TimerGroup, AnyTimer, PeriodicTimer},
};
use esp_println::println;
use esp_wifi::{initialize, wifi, EspWifiFor};
use ieee80211::{match_frames, mgmt_frame::BeaconFrame};

static KNOWN_SSIDS: Mutex<RefCell<BTreeSet<String>>> = Mutex::new(RefCell::new(BTreeSet::new()));

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
    let timer0: AnyTimer = timg0.timer0.into();
    let timer = PeriodicTimer::new(timer0);

    let init = initialize(
        EspWifiFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let wifi = peripherals.WIFI;

    // We must initialize some kind of interface and start it.
    let (_, mut controller) = wifi::new_with_mode(&init, wifi, wifi::WifiApDevice).unwrap();
    controller.start().unwrap();

    let mut sniffer = controller.take_sniffer().unwrap();
    sniffer.set_promiscuous_mode(true).unwrap();
    sniffer.set_receive_cb(|packet| {
        let _ = match_frames! {
            packet.data,
            beacon = BeaconFrame => {
                let Some(ssid) = beacon.ssid() else {
                    return;
                };
                if critical_section::with(|cs| {
                    KNOWN_SSIDS.borrow_ref_mut(cs).insert(ssid.to_string())
                }) {
                    println!("Found new AP with SSID: {ssid}");
                }
            }
        };
    });

    loop {}
}

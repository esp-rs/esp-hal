//! WiFi sniffer example
//!
//! Sniffs for beacon frames.
//!

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/sniffer esp-hal/unstable
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
use esp_hal::{clock::CpuClock, main, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::{init, wifi};
use ieee80211::{match_frames, mgmt_frame::BeaconFrame};

esp_bootloader_esp_idf::esp_app_desc!();

static KNOWN_SSIDS: Mutex<RefCell<BTreeSet<String>>> = Mutex::new(RefCell::new(BTreeSet::new()));

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // We must initialize some kind of interface and start it.
    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    controller.set_mode(wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();

    let mut sniffer = interfaces.sniffer;
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

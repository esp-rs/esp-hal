//! WiFi frame injection example
//!
//! Periodically transmits a beacon frame.

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/sniffer
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use core::marker::PhantomData;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    rng::Rng,
    timer::{timg::TimerGroup, AnyTimer, PeriodicTimer},
};
use esp_wifi::{initialize, wifi, EspWifiInitFor};
use ieee80211::{
    common::{CapabilitiesInformation, FCFFlags},
    element_chain,
    elements::{DSSSParameterSetElement, RawIEEE80211Element, SSIDElement},
    mgmt_frame::{body::BeaconBody, header::ManagementFrameHeader, BeaconFrame},
    scroll::Pwrite,
    supported_rates,
};

const SSID: &str = "esp-wifi 802.11 injection";
/// This is an arbitrary MAC address, used for the fake beacon frames.
const MAC_ADDRESS: [u8; 6] = [0x00, 0x80, 0x41, 0x13, 0x37, 0x42];

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::config::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let delay = Delay::new();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer = PeriodicTimer::new(timer0);

    let init = initialize(
        EspWifiInitFor::Wifi,
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

    // Create a buffer, which can hold the enitre serialized beacon frame.
    let mut beacon = [0u8; 300];
    let length = beacon
        .pwrite(
            BeaconFrame {
                header: ManagementFrameHeader {
                    fcf_flags: FCFFlags::new(),
                    duration: 0,
                    receiver_address: [0xff; 6].into(),
                    transmitter_address: MAC_ADDRESS.into(),
                    bssid: MAC_ADDRESS.into(),
                    ..Default::default()
                },
                body: BeaconBody {
                    timestamp: 0,
                    // We transmit a beacon every 100 ms/TUs
                    beacon_interval: 100,
                    capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                    elements: element_chain! {
                        SSIDElement::new(SSID).unwrap(),
                        // These are known good values.
                        supported_rates![
                            1 B,
                            2 B,
                            5.5 B,
                            11 B,
                            6,
                            9,
                            12,
                            18
                        ],
                        DSSSParameterSetElement {
                            current_channel: 1,
                        },
                        // This contains the Traffic Indication Map(TIM), for which `ieee80211-rs` currently lacks support.
                        RawIEEE80211Element {
                            tlv_type: 5,
                            slice: [0x01, 0x02, 0x00, 0x00].as_slice(),
                            _phantom: PhantomData
                        }
                    },
                    _phantom: PhantomData,
                },
            },
            0,
        )
        .unwrap();
    // Only use the actually written bytes.
    let beacon = &beacon[..length];

    loop {
        sniffer.send_raw_frame(false, beacon, false).unwrap();
        delay.delay(100.millis());
    }
}

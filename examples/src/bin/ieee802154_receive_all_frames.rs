//% CHIPS: esp32c6 esp32h2
//% FEATURES: esp-hal/unstable esp-radio/ieee802154 esp-radio/unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::main;
use esp_println::println;
use esp_radio::ieee802154::{Config, Ieee802154};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154);

    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: true,
        rx_when_idle: true,
        auto_ack_rx: false,
        auto_ack_tx: false,
        ..Default::default()
    });

    println!("Start receiving:");
    ieee802154.start_receive();

    loop {
        if let Some(frame) = ieee802154.received() {
            println!("Received {:?}\n", &frame);
        }
    }
}

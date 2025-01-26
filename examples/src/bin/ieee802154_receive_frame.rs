//% CHIPS: esp32c6 esp32h2
//% FEATURES: esp-ieee802154 esp-hal/unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::main;
use esp_ieee802154::{Config, Ieee802154};
use esp_println::println;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154, peripherals.RADIO_CLK);

    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: false,
        rx_when_idle: true,
        auto_ack_rx: true,
        auto_ack_tx: true,
        pan_id: Some(0x4242),
        short_addr: Some(0x2323),
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

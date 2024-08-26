//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::prelude::*;
use esp_ieee802154::*;
use esp_println::println;

#[entry]
fn main() -> ! {
    let (mut peripherals, _clocks) = esp_hal::init(CpuClock::boot_default());
    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154, &mut peripherals.RADIO_CLK);

    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: false,
        rx_when_idle: true,
        auto_ack_rx: true,
        auto_ack_tx: true,
        pan_id: Some(0x4242),
        short_addr: Some(0x2323),
        ..Config::default()
    });

    println!("Start receiving:");
    ieee802154.start_receive();

    loop {
        if let Some(frame) = ieee802154.get_received() {
            println!("Received {:?}\n", &frame);
        }
    }
}

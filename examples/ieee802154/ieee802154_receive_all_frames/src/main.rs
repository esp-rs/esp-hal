//% CHIP_FILTER: ieee802154_driver_supported
#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::main;
use esp_radio::ieee802154::{Config, Ieee802154};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32c5" => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        any(feature = "esp32h2", feature = "esp32h4") => peripherals.GPIO24,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    esp_alloc::heap_allocator!(size: 24 * 1024);

    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154);

    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: true,
        rx_when_idle: true,
        auto_ack_rx: false,
        auto_ack_tx: false,
        ..Default::default()
    });

    log::info!("Start receiving:");
    ieee802154.start_receive();

    loop {
        if let Some(frame) = ieee802154.received() {
            log::info!("Received {:?}\n", &frame);
        }
    }
}

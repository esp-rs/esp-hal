//% CHIP_FILTER: ieee802154_driver_supported
#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, main};
use esp_radio::ieee802154::{Config, Frame, Ieee802154};
use ieee802154::mac::{
    Address,
    FrameContent,
    FrameType,
    FrameVersion,
    Header,
    PanId,
    ShortAddress,
};

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

    let delay = Delay::new();

    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154);

    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: false,
        pan_id: Some(0x4242),
        short_addr: Some(0x2222),
        ..Default::default()
    });

    let mut seq_number = 0u8;
    loop {
        ieee802154
            .transmit(
                &Frame {
                    header: Header {
                        frame_type: FrameType::Data,
                        frame_pending: false,
                        ack_request: true,
                        pan_id_compress: false,
                        seq_no_suppress: false,
                        ie_present: false,
                        version: FrameVersion::Ieee802154_2003,
                        seq: seq_number,
                        destination: Some(Address::Short(PanId(0x4242), ShortAddress(0x2323))),
                        source: None,
                        auxiliary_security_header: None,
                    },
                    content: FrameContent::Data,
                    payload: b"Hello World".to_vec(),
                    footer: [0u8; 2],
                },
                true,
            )
            .ok();

        log::info!("Send frame with sequence number {seq_number}");
        delay.delay_millis(1000u32);
        seq_number = seq_number.wrapping_add(1);
    }
}

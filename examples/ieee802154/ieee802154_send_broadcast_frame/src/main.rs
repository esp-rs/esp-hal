#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, main};
use esp_println::println;
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
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 24 * 1024);

    let delay = Delay::new();

    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154);

    ieee802154.set_config(Config {
        channel: 15,
        promiscuous: false,
        pan_id: Some(0x4242),
        short_addr: Some(0x2323),
        ..Default::default()
    });

    let mut seq_number = 0u8;
    loop {
        ieee802154
            .transmit(&Frame {
                header: Header {
                    frame_type: FrameType::Data,
                    frame_pending: false,
                    ack_request: false,
                    pan_id_compress: false,
                    seq_no_suppress: false,
                    ie_present: false,
                    version: FrameVersion::Ieee802154_2003,
                    seq: seq_number,
                    destination: Some(Address::Short(PanId(0xffff), ShortAddress(0xffff))),
                    source: None,
                    auxiliary_security_header: None,
                },
                content: FrameContent::Data,
                payload: b"Hello World".to_vec(),
                footer: [0u8; 2],
            })
            .ok();

        println!("Send frame with sequence number {seq_number}");
        delay.delay_millis(1000u32);
        seq_number = seq_number.wrapping_add(1);
    }
}

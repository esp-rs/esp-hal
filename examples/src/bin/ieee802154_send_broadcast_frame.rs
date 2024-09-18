//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*};
use esp_ieee802154::{Config, Frame, Ieee802154};
use esp_println::println;
use ieee802154::mac::{
    Address,
    FrameContent,
    FrameType,
    FrameVersion,
    Header,
    PanId,
    ShortAddress,
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154, peripherals.RADIO_CLK);

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
                payload: heapless::Vec::from_slice(b"Hello World").unwrap(),
                footer: [0u8; 2],
            })
            .ok();

        println!("Send frame with sequence number {seq_number}");
        delay.delay_millis(1000u32);
        seq_number = seq_number.wrapping_add(1);
    }
}

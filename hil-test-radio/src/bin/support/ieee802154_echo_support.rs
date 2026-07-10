//! IEEE 802.15.4 echo support firmware.
//!
//! Runs on the second board of the radio HIL rig. It joins the same channel
//! and PAN as the device-under-test, auto-acknowledges frames addressed to it,
//! and echoes every received data frame's payload back to the DUT. This lets
//! the DUT test both the transmit (ACK) and receive paths against real peer
//! hardware.

//% CHIP_FILTER(has_ieee802154): esp32c6
//% SUPPORT-FIRMWARE: true
//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_ieee802154): esp-radio/ieee802154 esp-radio esp-radio-unstable
//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

use esp_hal::{clock::CpuClock, main};
use esp_radio::ieee802154::{Config, Frame, Ieee802154};
use hil_test as _;
use hil_test::ieee802154_params::{CHANNEL, DUT_ADDRESS, PAN_ID, SUPPORT_ADDRESS};
use ieee802154::mac::{
    Address,
    FrameContent,
    FrameType,
    FrameVersion,
    Header,
    PanId,
    ShortAddress,
};
use semihosting as _;

extern crate alloc;

fn init_heap() {
    cfg_select! {
        any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6) => {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        },
        any(esp32c5, esp32h2) => {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        },
        _ => {},
    }
}

fn echo_frame(seq: u8, payload: alloc::vec::Vec<u8>) -> Frame {
    Frame {
        header: Header {
            frame_type: FrameType::Data,
            frame_pending: false,
            ack_request: false,
            pan_id_compress: false,
            seq_no_suppress: false,
            ie_present: false,
            version: FrameVersion::Ieee802154_2003,
            seq,
            destination: Some(Address::Short(PanId(PAN_ID), ShortAddress(DUT_ADDRESS))),
            source: None,
            auxiliary_security_header: None,
        },
        content: FrameContent::Data,
        payload,
        footer: [0u8; 2],
    }
}

#[main]
fn main() -> ! {
    init_heap();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut ieee802154 = Ieee802154::new(peripherals.IEEE802154);
    ieee802154.set_config(Config {
        channel: CHANNEL,
        pan_id: Some(PAN_ID),
        short_addr: Some(SUPPORT_ADDRESS),
        rx_when_idle: true,
        auto_ack_rx: true,
        auto_ack_tx: true,
        ..Default::default()
    });

    ieee802154.start_receive();

    // Tell the HIL runner we are on-air and ready to service the DUT.
    hil_test::signal_harness_ready();

    let mut seq = 0u8;
    loop {
        if let Some(Ok(received)) = ieee802154.received() {
            // Echo the payload straight back to the DUT. `transmit` is deferred
            // internally if an auto-ACK is still in flight, so it is safe to
            // call right after receiving.
            let response = echo_frame(seq, received.frame.payload);
            ieee802154.transmit(&response, false).ok();
            seq = seq.wrapping_add(1);
        }
    }
}

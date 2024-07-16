//! This example sends a CAN message to another ESP and receives it back.
//!
//! This example works without CAN Transceivers by:
//! * setting the tx pins to open drain
//! * connecting all rx and tx pins together
//! * adding a pull-up to the signal pins
//!
//! The following wiring is assumed:
//! - TX => GPIO0
//! - RX => GPIO2
//!
//! ESP1/GND --- ESP2/GND
//! ESP1/GPIO0 --- ESP1/GPIO2 --- ESP2/GPIO0 --- ESP2/GPIO2 --- 4.8kOhm --- ESP1/5V
//!
//! `IS_FIRST_SENDER` below must be set to false on one of the ESP's

//% CHIPS: esp32c3 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

const IS_FIRST_SENDER: bool = true;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    twai::{self, filter::SingleStandardFilter, EspTwaiFrame, StandardId},
};
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let can_tx_pin = io.pins.gpio0;
    let can_rx_pin = io.pins.gpio2;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // !!! Use `new` when using a transceiver. `new_no_transceiver` sets TX to open-drain

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new_no_transceiver(
        peripherals.TWAI0,
        can_tx_pin,
        can_rx_pin,
        &clocks,
        CAN_BAUDRATE,
    );

    // Partially filter the incoming messages to reduce overhead of receiving
    // undesired messages. Note that due to how the hardware filters messages,
    // standard ids and extended ids may both match a filter. Frame ids should
    // be explicitly checked in the application instead of fully relying on
    // these partial acceptance filters to exactly match.
    // A filter that matches StandardId::ZERO.
    const FILTER: SingleStandardFilter =
        SingleStandardFilter::new(b"00000000000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let mut can = can_config.start();

    if IS_FIRST_SENDER {
        // Send a frame to the other ESP
        let frame = EspTwaiFrame::new(StandardId::ZERO.into(), &[1, 2, 3]).unwrap();
        block!(can.transmit(&frame)).unwrap();
        println!("Sent a frame");
    }

    // Wait for a frame to be received.
    let frame = block!(can.receive()).unwrap();

    println!("Received a frame: {frame:?}");

    if !IS_FIRST_SENDER {
        // Transmit the frame back to the other ESP
        block!(can.transmit(&frame)).unwrap();
        println!("Sent a frame");
    }

    loop {}
}

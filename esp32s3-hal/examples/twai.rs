//! This example sends a CAN message to another ESP and receives it back.
//!
//! Wiring:
//! This example works without CAN Transceivers by:
//! * setting the tx pins to open drain
//! * connecting all rx and tx pins together
//! * adding a pull-up to the signal pins
//!
//! ESP1/GND --- ESP2/GND
//! ESP1/IO2 --- ESP1/IO3 --- ESP2/IO2 --- ESP2/IO3 --- 4.8kOhm --- ESP1/5V
//!
//! `IS_FIRST_SENDER` below must be set to false on one of the ESP's

#![no_std]
#![no_main]

const IS_FIRST_SENDER: bool = true;

// Run this example with the eh1 feature enabled to use embedded-can instead of
// embedded-hal-0.2.7. embedded-can was split off from embedded-hal before it's
// upgrade to 1.0.0. cargo run --example twai --features eh1 --release
#[cfg(feature = "eh1")]
use embedded_can::{nb::Can, Frame, StandardId};
// Run this example without the eh1 flag to use the embedded-hal 0.2.7 CAN traits.
// cargo run --example twai --release
#[cfg(not(feature = "eh1"))]
use embedded_hal::can::{Can, Frame, StandardId};
use esp32s3_hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, prelude::*, twai};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set the tx pin as open drain. Skip this if using transceivers.
    let can_tx_pin = io.pins.gpio2.into_open_drain_output();
    let can_rx_pin = io.pins.gpio3;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new(
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
    const FILTER: twai::filter::SingleStandardFilter =
        twai::filter::SingleStandardFilter::new(b"00000000000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let mut can = can_config.start();

    if IS_FIRST_SENDER {
        // Send a frame to the other ESP
        let frame = Frame::new(StandardId::ZERO, &[1, 2, 3]).unwrap();
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

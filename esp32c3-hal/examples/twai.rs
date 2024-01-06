//! This example demonstrates the use of the twai peripheral to send and receive
//! frames. When `IS_SENDER` is set to `true`, the node will send a frame
//! every `DELAY_MS` interval. When `IS_SENDER` is set to `false`, the node will
//! wait to receive a frame and repeat it back.
//!
//! When running this example on two ESP boards, `IS_SENDER` must be set to
//! `true` for at least one node. It is okay to have multiple senders.

#![no_std]
#![no_main]

const IS_SENDER: bool = true;
const DELAY_MS: u32 = 1000;

// Run this example with the eh1 feature enabled to use embedded-can instead of
// embedded-hal-0.2.7. embedded-can was split off from embedded-hal before it's
// upgrade to 1.0.0. cargo run --example twai --features eh1 --release
#[cfg(feature = "eh1")]
use embedded_can::{Frame, Id};
// Run this example without the eh1 flag to use the embedded-hal 0.2.7 CAN traits.
// cargo run --example twai --release
#[cfg(not(feature = "eh1"))]
use embedded_hal::can::{Frame, Id, StandardId};
use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    twai::{self, EspTwaiError, EspTwaiFrame},
    Delay,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut delay = Delay::new(&clocks);

    // Use GPIO pins 2 and 3 to connect to the respective pins on the CAN
    // transceiver.
    let can_tx_pin = io.pins.gpio2;
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

    let mut counter: u32 = 0;

    loop {
        if IS_SENDER {
            // If this is the sender-node, then send a frame every DELAY_MS inteval.

            // Send a frame to the other ESP
            let frame = Frame::new(StandardId::ZERO, &counter.to_be_bytes()).unwrap();

            // Check for BusOff error, and restart the peripheral if it occurs.
            let transmit_result = block!(can.transmit(&frame));
            if let Err(EspTwaiError::BusOff) = transmit_result {
                println!("Transmit Error: BusOff. restarting...");
                let config = can.stop();
                can = config.start();
                continue;
            }

            println!("Transmitted a frame.");
            print_frame(&frame);
            counter += 1;

            // Check for received frame, but do not block.
            let receive_result = can.receive();
            if let Ok(f) = receive_result {
                println!("Received a frame:");
                print_frame(&f);
            }

            delay.delay_ms(DELAY_MS);
        } else {
            // If this is the receiver-node, then wait for a frame to be received, then send
            // it back

            // Wait for a frame to be received.
            let frame = block!(can.receive()).unwrap();

            println!("Received a frame:");
            print_frame(&frame);

            // Transmit the frame back.
            // We just received a frame, so chances of BusOff are low.
            block!(can.transmit(&frame)).unwrap();

            println!("Transmitted the frame back.");
        }
    }
}

fn print_frame(frame: &EspTwaiFrame) {
    // Print different messages based on the frame id type.
    match frame.id() {
        Id::Standard(id) => {
            println!("\tStandard Id: {:?}", id);
        }
        Id::Extended(id) => {
            println!("\tExtended Id: {:?}", id);
        }
    }

    // Print out the frame data or the requested data length code for a remote
    // transmission request frame.
    if frame.is_data_frame() {
        println!("\tData: {:?}", frame.data());
    } else {
        println!("\tRemote Frame. Data Length Code: {}", frame.dlc());
    }
}

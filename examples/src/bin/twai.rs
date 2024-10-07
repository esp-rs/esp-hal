//! This example sends a TWAI message to another ESP and receives it back.
//!
//! `IS_FIRST_SENDER` below must be set to false on one of the ESP's
//!
//! In case you want to use `self-testing`, get rid of everything related to the aforementioned `IS_FIRST_SENDER`
//! and follow the advice in the comments related to this mode.
//!
//! The following wiring is assumed:
//! - TX/RX => GPIO2, connected internally and with internal pull-up resistor.
//!
//! ESP1/GND --- ESP2/GND
//! ESP1/GPIO2 --- ESP2/GPIO2
//!
//! Notes for external transciever use:
//!
//! The default setup assumes that two microcontrollers are connected directly without an external
//! transceiver. If you want to use an external transceiver, you need to:
//! * uncomment the `rx_pin` line
//! * use `new()` function to create the TWAI configuration.
//! * change the `tx_pin` and `rx_pin` to the appropriate pins for your boards.

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

const IS_FIRST_SENDER: bool = true;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::Io,
    prelude::*,
    twai::{self, filter::SingleStandardFilter, EspTwaiFrame, StandardId, TwaiMode},
};
use esp_println::println;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let tx_pin = io.pins.gpio2;
    // let rx_pin = io.pins.gpio0; // Uncomment if you want to use an external transciever.

    // Without an external transciever, we only need a single line between the two MCUs.
    let rx_pin = tx_pin.peripheral_input(); // Comment this line if you want to use an external transciever.

    // The speed of the CAN bus.
    const TWAI_BAUDRATE: twai::BaudRate = twai::BaudRate::B125K;

    // !!! Use `new` when using a transceiver. `new_no_transceiver` sets TX to open-drain
    // Self-testing also works using the regular `new` function.

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    // For self-testing use `SelfTest` mode of the TWAI peripheral.
    let mut twai_config = twai::TwaiConfiguration::new_no_transceiver(
        peripherals.TWAI0,
        rx_pin,
        tx_pin,
        TWAI_BAUDRATE,
        TwaiMode::Normal,
    );

    // Partially filter the incoming messages to reduce overhead of receiving
    // undesired messages. Note that due to how the hardware filters messages,
    // standard ids and extended ids may both match a filter. Frame ids should
    // be explicitly checked in the application instead of fully relying on
    // these partial acceptance filters to exactly match.
    // A filter that matches StandardId::ZERO.
    twai_config.set_filter(
        const { SingleStandardFilter::new(b"00000000000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
    );

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let mut can = twai_config.start();

    loop {
        // Send a frame to the other ESP
        // Use `new_self_reception` if you want to use self-testing.
        let frame = EspTwaiFrame::new(StandardId::ZERO, &[1, 2, 3]).unwrap();
        println!("Sending a frame");
        block!(can.transmit(&frame)).unwrap();
        println!("Sent a frame");

        Delay::new().delay_millis(500u32);
    }
}

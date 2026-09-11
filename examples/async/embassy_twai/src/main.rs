//! embassy twai
//!
//! This is an example of running the embassy executor and asynchronously
//! sending and receiving TWAI messages between two ESPs.
//!
//! `IS_FIRST_SENDER` below must be set to false on one of the ESP's
//!
//! In case you want to use `self-testing`, get rid of everything related to the
//! aforementioned `IS_FIRST_SENDER` and follow the advice in the comments
//! related to this mode.
//!
//! The following wiring is assumed:
//! - TX/RX => GPIO2, connected internally and with internal pull-up resistor.
//!
//! ESP1/GND --- ESP2/GND
//! ESP1/GPIO2 --- ESP2/GPIO2
//!
//! With no peer on the bus, transmissions fail: once the peripheral reaches
//! the error-passive state, `transmit_async` gives up on the frame and returns
//! `EspTwaiError::TransmissionAborted` instead of retrying forever.
//!
//! Notes for external transceiver use:
//!
//! The default setup assumes that two microcontrollers are connected directly
//! without an external transceiver. If you want to use an external transceiver,
//! you need to:
//! * uncomment the `rx_pin` line
//! * use `new()` function to create the TWAI configuration.
//! * change the `tx_pin` and `rx_pin` to the appropriate pins for your boards.

//% CHIP_FILTER: twai_driver_supported

#![no_std]
#![no_main]

const IS_FIRST_SENDER: bool = true;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    timer::timg::TimerGroup,
    twai::{self, EspTwaiFrame, StandardId, TwaiMode, filter::SingleStandardFilter},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Without an external transceiver, we only need a single line between the two
    // MCUs.
    let (rx_pin, tx_pin) = unsafe { peripherals.GPIO2.split() };
    // Use these if you want to use an external transceiver:
    // let tx_pin = peripherals.GPIO2;
    // let rx_pin = peripherals.GPIO0;

    // The speed of the bus.
    const TWAI_BAUDRATE: twai::BaudRate = twai::BaudRate::B125K;

    // !!! Use `new` when using a transceiver. `new_no_transceiver` sets TX to
    // open-drain Self-testing also works using the regular `new` function.

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
        const { SingleStandardFilter::new(b"xxxxxxxxxx1", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
    );

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let mut twai = twai_config.into_async().start();

    if IS_FIRST_SENDER {
        // Send a frame to the other ESP
        // Use `new_self_reception` if you want to use self-testing.
        let frame = EspTwaiFrame::new(StandardId::ZERO, &[1, 2, 3]).unwrap();
        match twai.transmit_async(&frame).await {
            Ok(()) => println!("Sent a frame"),
            Err(e) => println!("Error sending a frame: {e:?}"),
        }
    }

    loop {
        // Wait for a frame to be received.
        match twai.receive_async().await {
            Ok(frame) => println!("Received a frame: {frame:?}"),
            Err(e) => {
                println!("Error receiving a frame: {e:?}");
                continue;
            }
        }

        Timer::after(Duration::from_millis(250)).await;

        let frame = EspTwaiFrame::new(StandardId::ZERO, &[1, 2, 3]).unwrap();
        // Transmit a new frame back to the other ESP
        match twai.transmit_async(&frame).await {
            Ok(()) => println!("Sent a frame"),
            Err(e) => println!("Error sending a frame: {e:?}"),
        }
    }
}

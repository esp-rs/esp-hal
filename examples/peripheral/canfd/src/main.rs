//! This example sends a CAN FD frame to another ESP and receives it back.
//!
//! `IS_FIRST_SENDER` below must be set to false on one of the ESPs.
//!
//! The following wiring is assumed:
//! - TX/RX => GPIO2, connected internally and with a pull-up resistor.
//!
//! ESP1/GND --- ESP2/GND
//! ESP1/GPIO2 --- ESP2/GPIO2
//!
//! Without a transceiver, the two controllers drive one wire open-drain and a
//! pull-up makes it recessive. The internal pull-up is enough for the 500 kbit/s
//! arbitration phase, but the rising edges of the 2 Mbit/s data phase need an
//! external pull-up of a few kilo-ohms to 3V3.
//!
//! Notes for external transceiver use:
//!
//! The default setup assumes that two microcontrollers are connected directly
//! without an external transceiver. If you want to use an external transceiver,
//! you need to:
//! * remove the `with_no_transceiver` call from the configuration
//! * use separate pins for `rx_pin` and `tx_pin`, and change them to the appropriate pins for your
//!   boards.

//% CHIP_FILTER: canfd_driver_supported

#![no_std]
#![no_main]

const IS_FIRST_SENDER: bool = true;

use esp_backtrace as _;
use esp_hal::{
    canfd::{CanFd, Config, Frame, TxBufferState},
    delay::Delay,
    main,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Without an external transceiver, we only need a single line between the two
    // MCUs.
    let (rx_pin, tx_pin) = unsafe { peripherals.GPIO2.split() };
    // Use these if you want to use an external transceiver:
    // let tx_pin = peripherals.GPIO2;
    // let rx_pin = peripherals.GPIO0;

    // The default configuration runs the arbitration phase at 500 kbit/s and
    // the data phase at 2 Mbit/s from the 80 MHz function clock. Other bit
    // rates are set with `with_nominal_timing` and `with_fd_timing`.
    let config = Config::default().with_no_transceiver(true);

    // The controller is configured but stays off the bus until it is started.
    let mut canfd = CanFd::new(peripherals.TWAI0, config)
        .unwrap()
        .with_rx(rx_pin)
        .with_tx(tx_pin);

    // Join the bus. This waits for the bus to be idle long enough for the
    // controller to become error-active.
    canfd.start().unwrap();

    // A CAN FD frame with 64 bytes of payload, sent with the data phase at the
    // FD bit rate.
    let payload: [u8; 64] = core::array::from_fn(|i| i as u8);
    let frame = Frame::new_fd(0x123, false, true, &payload).unwrap();

    if IS_FIRST_SENDER {
        send(&mut canfd, &frame);
    }

    let delay = Delay::new();
    loop {
        // Wait for a frame to be received.
        while canfd.rx_frame_count() == 0 {}
        let received = canfd.receive().unwrap();

        println!("Received a frame: {received:?}");
        delay.delay_millis(250);

        // Transmit a new frame back to the other ESP.
        send(&mut canfd, &frame);
    }
}

/// Queues a frame and waits until the hardware is done with it.
fn send(canfd: &mut CanFd<'_>, frame: &Frame) {
    let index = canfd.transmit(frame).unwrap();
    loop {
        match canfd.tx_buffer_state(index) {
            TxBufferState::Ready | TxBufferState::InProgress => {}
            TxBufferState::Ok => break,
            state => panic!("transmission failed: {state:?}"),
        }
    }
    println!("Sent a frame");
}

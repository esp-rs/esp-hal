//! This example demonstrates use of the TWAI peripheral running the embassy
//! executor and asynchronously receiving and transmitting TWAI frames.
//!
//! The `receiver` task waits to receive a frame and puts it into a channel
//! which will be picked up by the `transmitter` task.
//!
//! The `transmitter` task waits for a channel to receive a frame and transmits
//! it.
//!
//! This example should work with another ESP board running the `twai` example
//! with `IS_FIRST_SENDER` set to `true`.
//!
//! The following wiring is assumed:
//! - TX/RX => GPIO2, connected internally and with internal pull-up resistor.
//!
//! ESP1/GND --- ESP2/GND
//! ESP1/GPIO2 --- ESP2/GPIO2
//!
//! Notes for external transceiver use:
//!
//! The default setup assumes that two microcontrollers are connected directly without an external
//! transceiver. If you want to use an external transceiver, you need to:
//! * uncomment the `rx_pin` line
//! * use `new()` function to create the TWAI configuration.
//! * change the `tx_pin` and `rx_pin` to the appropriate pins for your boards.

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embedded_can::{Frame, Id};
use esp_backtrace as _;
use esp_hal::{
    timer::timg::TimerGroup,
    twai::{self, EspTwaiFrame, StandardId, TwaiMode, TwaiRx, TwaiTx},
};
use esp_println::println;
use static_cell::StaticCell;

type TwaiOutbox = Channel<NoopRawMutex, EspTwaiFrame, 16>;

#[embassy_executor::task]
async fn receiver(mut rx: TwaiRx<'static, esp_hal::Async>, channel: &'static TwaiOutbox) -> ! {
    loop {
        let frame = rx.receive_async().await;

        match frame {
            Ok(frame) => {
                println!("Received a frame:");
                print_frame(&frame);

                // Send a response
                let frame =
                    EspTwaiFrame::new(StandardId::new(1).unwrap(), &[4, 5, 6, 7, 8]).unwrap();
                channel.send(frame).await;
            }
            Err(e) => {
                println!("Receive error: {:?}", e);
            }
        }
    }
}

#[embassy_executor::task]
async fn transmitter(mut tx: TwaiTx<'static, esp_hal::Async>, channel: &'static TwaiOutbox) -> ! {
    loop {
        let frame = channel.receive().await;
        let result = tx.transmit_async(&frame).await;

        match result {
            Ok(()) => {
                println!("Transmitted a frame:");
                print_frame(&frame);
            }
            Err(e) => {
                println!("Transmit error: {:?}", e);
            }
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Without an external transceiver, we only need a single line between the two MCUs.
    let (rx_pin, tx_pin) = peripherals.GPIO2.split();
    // Use these if you want to use an external transceiver:
    // let tx_pin = peripherals.GPIO2;
    // let rx_pin = peripherals.GPIO0;

    // The speed of the bus.
    const TWAI_BAUDRATE: twai::BaudRate = twai::BaudRate::B125K;

    // !!! Use `new` when using a transceiver. `new_no_transceiver` sets TX to open-drain

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut twai_config = twai::TwaiConfiguration::new_no_transceiver(
        peripherals.TWAI0,
        rx_pin,
        tx_pin,
        TWAI_BAUDRATE,
        TwaiMode::Normal,
    )
    .into_async();

    // Partially filter the incoming messages to reduce overhead of receiving
    // undesired messages. Note that due to how the hardware filters messages,
    // standard ids and extended ids may both match a filter. Frame ids should
    // be explicitly checked in the application instead of fully relying on
    // these partial acceptance filters to exactly match. A filter that matches
    // standard ids of an even value.
    const FILTER: twai::filter::SingleStandardFilter =
        twai::filter::SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    twai_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let twai = twai_config.start();

    // Get separate transmit and receive halves of the peripheral.
    let (rx, tx) = twai.split();

    static CHANNEL: StaticCell<TwaiOutbox> = StaticCell::new();
    let channel = &*CHANNEL.init(Channel::new());

    spawner.spawn(receiver(rx, channel)).ok();
    spawner.spawn(transmitter(tx, channel)).ok();
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

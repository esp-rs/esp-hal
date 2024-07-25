//! This example demonstrates use of the twai peripheral running the embassy
//! executor and asynchronously receiving and transmitting twai frames.
//!
//! The `receiver` task waits to receive a frame and puts it into a channel
//! which will be picked up by the `transmitter` task.
//!
//! The `transmitter` task waits for a channel to receive a frame and transmits
//! it.
//!
//! This example should work with another ESP board running the `twai` example
//! with `IS_SENDER` set to `true`.
//!
//! The following wiring is assumed:
//! - TX => GPIO0
//! - RX => GPIO2

//% CHIPS: esp32c3 esp32c6 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embedded_can::{Frame, Id};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    interrupt,
    peripherals::{self, Peripherals, TWAI0},
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
    twai::{self, EspTwaiFrame, TwaiRx, TwaiTx},
};
use esp_println::println;
use static_cell::StaticCell;

type TwaiOutbox = Channel<NoopRawMutex, EspTwaiFrame, 16>;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task]
async fn receiver(
    mut rx: TwaiRx<'static, TWAI0, esp_hal::Async>,
    channel: &'static TwaiOutbox,
) -> ! {
    loop {
        let frame = rx.receive_async().await;

        match frame {
            Ok(frame) => {
                println!("Received a frame:");
                print_frame(&frame);

                // repeat the frame back
                channel.send(frame).await;
            }
            Err(e) => {
                println!("Receive error: {:?}", e);
            }
        }
    }
}

#[embassy_executor::task]
async fn transmitter(
    mut tx: TwaiTx<'static, TWAI0, esp_hal::Async>,
    channel: &'static TwaiOutbox,
) -> ! {
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
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let can_tx_pin = io.pins.gpio0;
    let can_rx_pin = io.pins.gpio2;

    // The speed of the CAN bus.
    const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

    // !!! Use `new_async` when using a transceiver. `new_async_no_transceiver` sets TX to open-drain

    // Begin configuring the TWAI peripheral. The peripheral is in a reset like
    // state that prevents transmission but allows configuration.
    let mut can_config = twai::TwaiConfiguration::new_async_no_transceiver(
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
    // these partial acceptance filters to exactly match. A filter that matches
    // standard ids of an even value.
    const FILTER: twai::filter::SingleStandardFilter =
        twai::filter::SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
    can_config.set_filter(FILTER);

    // Start the peripheral. This locks the configuration settings of the peripheral
    // and puts it into operation mode, allowing packets to be sent and
    // received.
    let can = can_config.start();

    // Get separate transmit and receive halves of the peripheral.
    let (tx, rx) = can.split();

    interrupt::enable(
        peripherals::Interrupt::TWAI0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

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

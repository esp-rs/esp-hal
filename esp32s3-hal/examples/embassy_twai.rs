// ! embassy twai
// !
// ! This is an example of running the embassy executor and asynchronously
// ! receiving and transmitting twai frames.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use esp32s3_hal::{
    clock::ClockControl,
    embassy,
    interrupt,
    peripherals::{self, Peripherals, TWAI0},
    prelude::*,
    twai::{self, EspTwaiFrame, TwaiRx, TwaiTx},
    IO,
};
use esp_backtrace as _;
use static_cell::make_static;

type TwaiOutbox = Channel<NoopRawMutex, EspTwaiFrame, 16>;

#[embassy_executor::task]
async fn receiver(mut rx: TwaiRx<'static, TWAI0>, channel: &'static TwaiOutbox) -> ! {
    loop {
        // let frame = nb::block!(can.receive());
        let frame = rx.receive_async().await;

        match frame {
            Ok(frame) => {
                esp_println::println!("Received frame: {:?}", frame);
                // repeat the frame back
                channel.send(frame).await;
            }
            Err(e) => {
                esp_println::println!("Receive error: {:?}", e);
            }
        }
    }
}

#[embassy_executor::task]
async fn transmitter(mut tx: TwaiTx<'static, TWAI0>, channel: &'static TwaiOutbox) -> ! {
    loop {
        let frame = channel.receive().await;
        let result = tx.transmit_async(&frame).await;

        match result {
            Ok(()) => {
                esp_println::println!("Transmitted frame: {:?}", frame);
            }
            Err(e) => {
                esp_println::println!("Transmit error: {:?}", e);
            }
        }
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32c3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32c3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

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

    let channel = &*make_static!(Channel::new());

    spawner.spawn(receiver(rx, channel)).ok();
    spawner.spawn(transmitter(tx, channel)).ok();
}

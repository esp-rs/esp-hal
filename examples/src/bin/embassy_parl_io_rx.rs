//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO1, GPIO2, GPIO3, and GPIO4.

//% CHIPS: esp32c6 esp32h2
//% FEATURES: embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma::Dma,
    dma_buffers,
    gpio::NoPin,
    parl_io::{BitPackOrder, ParlIoRxOnly, RxFourBits},
    prelude::*,
    timer::systimer::{SystemTimer, Target},
};
use esp_println::println;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(systimer.alarm0);

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);

    let mut rx_pins = RxFourBits::new(
        peripherals.GPIO1,
        peripherals.GPIO2,
        peripherals.GPIO3,
        peripherals.GPIO4,
    );
    let mut rx_clk_pin = NoPin;

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma.channel0.into_async(),
        rx_descriptors,
        1.MHz(),
    )
    .unwrap();

    let mut parl_io_rx = parl_io
        .rx
        .with_config(
            &mut rx_pins,
            &mut rx_clk_pin,
            BitPackOrder::Msb,
            Some(0xfff),
        )
        .unwrap();

    let buffer = rx_buffer;
    loop {
        parl_io_rx.read_dma_async(buffer).await.unwrap();
        println!(
            "Received: {:02x?} ... {:02x?}",
            &buffer[..30],
            &buffer[(buffer.len() - 30)..]
        );

        Timer::after(Duration::from_millis(500)).await;
    }
}

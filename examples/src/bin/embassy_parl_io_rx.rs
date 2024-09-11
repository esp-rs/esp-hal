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
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    parl_io::{no_clk_pin, BitPackOrder, ParlIoRxOnly, RxFourBits},
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

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure_for_async(false, DmaPriority::Priority0),
        rx_descriptors,
        1.MHz(),
    )
    .unwrap();

    let mut parl_io_rx = parl_io
        .rx
        .with_config(&mut rx_pins, no_clk_pin(), BitPackOrder::Msb, Some(0xfff))
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

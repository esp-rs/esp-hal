//! This shows using Parallel IO to output 4 bit parallel data at 1MHz clock
//! rate.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO1, GPIO2, GPIO3, and GPIO4
//! - Valid pin => GPIO5 (driven high during an active transfer)
//! - Clock output pin => GPIO8
//!
//! You can use a logic analyzer to see how the pins are used.

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
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        ParlIoTxOnly,
        SampleEdge,
        TxFourBits,
        TxPinConfigWithValidPin,
    },
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

    let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let tx_pins = TxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let mut pin_conf = TxPinConfigWithValidPin::new(tx_pins, io.pins.gpio5);

    let parl_io = ParlIoTxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure_for_async(false, DmaPriority::Priority0),
        tx_descriptors,
        1.MHz(),
    )
    .unwrap();

    let mut clock_pin = ClkOutPin::new(io.pins.gpio8);

    let mut parl_io_tx = parl_io
        .tx
        .with_config(
            &mut pin_conf,
            &mut clock_pin,
            0,
            SampleEdge::Normal,
            BitPackOrder::Msb,
        )
        .unwrap();

    let buffer = tx_buffer;
    for i in 0..buffer.len() {
        buffer[i] = (i % 255) as u8;
    }

    loop {
        parl_io_tx.write_dma_async(buffer).await.unwrap();
        println!("Transferred {} bytes", buffer.len());

        Timer::after(Duration::from_millis(500)).await;
    }
}

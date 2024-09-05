//! This shows using Parallel IO to output 4 bit parallel data at 1MHz clock
//! rate.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO1, GPIO2, GPIO3, and GPIO4
//! - Valid pin => GPIO5 (driven high during an active transfer)
//! - Clock output pin => GPIO6
//!
//! You can use a logic analyzer to see how the pins are used.

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
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
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let (tx_buffer, tx_descriptors, _, _) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let tx_pins = TxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let mut pin_conf = TxPinConfigWithValidPin::new(tx_pins, io.pins.gpio5);

    let parl_io = ParlIoTxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(false, DmaPriority::Priority0),
        tx_descriptors,
        1.MHz(),
    )
    .unwrap();

    let mut clock_pin = ClkOutPin::new(io.pins.gpio6);

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

    let delay = Delay::new();

    loop {
        let transfer = parl_io_tx.write_dma(&buffer).unwrap();
        transfer.wait().unwrap();
        println!("Transferred {} bytes", buffer.len());

        delay.delay_millis(500);
    }
}

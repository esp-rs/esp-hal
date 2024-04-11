//! This shows using Parallel IO to output 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.
//! GPIO 5 as the "valid pin" (driven high during an active transfer) and GPIO
//! 6 as the clock signal output.
//!
//! You can use a logic analyzer to see how the pins are used.

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::IO,
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        ParlIoTxOnly,
        SampleEdge,
        TxFourBits,
        TxPinConfigWithValidPin,
    },
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let tx_pins = TxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let pin_conf = TxPinConfigWithValidPin::new(tx_pins, io.pins.gpio5);

    let parl_io = ParlIoTxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        1.MHz(),
        &clocks,
    )
    .unwrap();

    let clock_pin = ClkOutPin::new(io.pins.gpio6);

    let mut parl_io_tx = parl_io
        .tx
        .with_config(
            pin_conf,
            clock_pin,
            0,
            SampleEdge::Normal,
            BitPackOrder::Msb,
        )
        .unwrap();

    let buffer = tx_buffer;
    for i in 0..buffer.len() {
        buffer[i] = (i % 255) as u8;
    }

    let delay = Delay::new(&clocks);

    loop {
        let transfer = parl_io_tx.write_dma(&buffer).unwrap();
        transfer.wait().unwrap();
        println!("Transferred {} bytes", buffer.len());

        delay.delay_millis(500);
    }
}

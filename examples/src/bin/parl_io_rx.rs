//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! The following wiring is assumed:
//! - Data pins => GPIO1, GPIO2, GPIO3, and GPIO4.

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    parl_io::{no_clk_pin, BitPackOrder, ParlIoRxOnly, RxFourBits},
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(false, DmaPriority::Priority0),
        rx_descriptors,
        1.MHz(),
    )
    .unwrap();

    let mut parl_io_rx = parl_io
        .rx
        .with_config(&mut rx_pins, no_clk_pin(), BitPackOrder::Msb, Some(0xfff))
        .unwrap();

    let mut buffer = rx_buffer;
    buffer.fill(0u8);

    let delay = Delay::new();

    loop {
        let transfer = parl_io_rx.read_dma(&mut buffer).unwrap();
        transfer.wait().unwrap();
        println!("Received: {:02x?} ...", &buffer[..30]);

        delay.delay_millis(500);
    }
}

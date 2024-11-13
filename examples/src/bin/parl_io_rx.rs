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
    gpio::NoPin,
    parl_io::{BitPackOrder, ParlIoRxOnly, RxFourBits},
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32000, 0);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut rx_pins = RxFourBits::new(
        peripherals.GPIO1,
        peripherals.GPIO2,
        peripherals.GPIO3,
        peripherals.GPIO4,
    );
    let mut rx_clk_pin = NoPin;

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(false, DmaPriority::Priority0),
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

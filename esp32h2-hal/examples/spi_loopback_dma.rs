//! SPI loopback test using DMA
//!
//! Folowing pins are used:
//! SCLK    GPIO1
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO11
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

#![no_std]
#![no_main]

use esp32h2_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    Delay,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio1;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio3;
    let cs = io.pins.gpio11;

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(32000);

    let mut spi = Spi::new(peripherals.SPI2, 100u32.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
        .with_dma(dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));

    let mut delay = Delay::new(&clocks);

    // DMA buffer require a static life-time
    let mut send = tx_buffer;
    let mut receive = rx_buffer;
    let mut i = 0;

    for (i, v) in send.iter_mut().enumerate() {
        *v = (i % 255) as u8;
    }

    loop {
        send[0] = i;
        send[send.len() - 1] = i;
        i = i.wrapping_add(1);

        let transfer = spi.dma_transfer(send, receive).unwrap();
        // here we could do something else while DMA transfer is in progress
        // the buffers and spi is moved into the transfer and we can get it back via
        // `wait`
        (receive, send, spi) = transfer.wait().unwrap();
        println!(
            "{:x?} .. {:x?}",
            &receive[..10],
            &receive[receive.len() - 10..]
        );

        delay.delay_ms(250u32);
    }
}

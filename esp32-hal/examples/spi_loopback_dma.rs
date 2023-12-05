//! SPI loopback test using DMA
//!
//! Folowing pins are used:
//! SCLK    GPIO19
//! MISO    GPIO25
//! MOSI    GPIO23
//! CS      GPIO22
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    dma_buffers,
    gpio::IO,
    pdma::Dma,
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
    let sclk = io.pins.gpio19;
    let miso = io.pins.gpio25;
    let mosi = io.pins.gpio23;
    let cs = io.pins.gpio22;

    let dma = Dma::new(system.dma);
    let dma_channel = dma.spi2channel;

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
        let mut n = 0;
        // Check is_done until the transfer is almost done (32000 bytes at 100kHz is
        // 2.56 seconds), then move to wait().
        while !transfer.is_done() && n < 10 {
            delay.delay_ms(250u32);
            n += 1;
        }
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

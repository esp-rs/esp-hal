//! SPI loopback test using DMA
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO4
//! CS      GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
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
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;
    let cs = io.pins.gpio5;

    let dma = Dma::new(peripherals.DMA);

    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.spi2channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
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

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        // here we could do something else while DMA transfer is in progress
        let mut n = 0;
        // Check is_done until the transfer is almost done (32000 bytes at 100kHz is
        // 2.56 seconds), then move to wait().
        while !transfer.is_done() && n < 10 {
            delay.delay_ms(250u32);
            n += 1;
        }

        transfer.wait().unwrap();
        println!(
            "{:x?} .. {:x?}",
            &receive[..10],
            &receive[receive.len() - 10..]
        );

        delay.delay_ms(250u32);
    }
}

//! SPI loopback test using DMA
//!
//! The following wiring is assumed:
//! - SCLK => GPIO0
//! - MISO => GPIO2
//! - MOSI => GPIO4
//! - CS   => GPIO5
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
    delay::Delay,
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::Io,
    prelude::*,
    spi::{master::Spi, SpiMode},
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;
    let cs = io.pins.gpio5;

    let dma = Dma::new(peripherals.DMA);

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let dma_channel = dma.spi2channel;
        } else {
            let dma_channel = dma.channel0;
        }
    }

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
        .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
        .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

    let delay = Delay::new();

    let mut i = 0;

    for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
        *v = (i % 255) as u8;
    }

    loop {
        dma_tx_buf.as_mut_slice()[0] = i;
        *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i;
        i = i.wrapping_add(1);

        let transfer = spi
            .dma_transfer(dma_rx_buf, dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        // here we could do something else while DMA transfer is in progress
        let mut n = 0;
        // Check is_done until the transfer is almost done (32000 bytes at 100kHz is
        // 2.56 seconds), then move to wait().
        while !transfer.is_done() && n < 10 {
            delay.delay_millis(250);
            n += 1;
        }

        (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        println!(
            "{:x?} .. {:x?}",
            &dma_rx_buf.as_slice()[..10],
            &dma_rx_buf.as_slice().last_chunk::<10>().unwrap()
        );

        delay.delay_millis(250);
    }
}

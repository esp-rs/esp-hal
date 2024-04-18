//! SPI Full Duplex DMA Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO4
//! CS      GPIO5
//!
//! Connect MISO (GPIO2) and MOSI (GPIO4) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
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
};

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer() {
        const DMA_BUFFER_SIZE: usize = 4;

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

        let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) =
            dma_buffers!(DMA_BUFFER_SIZE);

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(
                false,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ));

        // DMA buffer require a static life-time
        let mut send = tx_buffer;
        let mut receive = rx_buffer;

        send.copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
        assert_eq!(send, receive);
    }

    #[test]
    #[timeout(3)]
    fn test_asymmetric_dma_transfer() {
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

        let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(4, 2);

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(
                false,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ));

        // DMA buffer require a static life-time
        let mut send = tx_buffer;
        let mut receive = rx_buffer;

        send.copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
        assert_eq!(send[0], receive[0]);
        assert_eq!(send[1], receive[1]);
        // Read the 2 remaining bytes so it does not cause issues in the next test.
        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer_huge_buffer() {
        const DMA_BUFFER_SIZE: usize = 4096;

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

        let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) =
            dma_buffers!(DMA_BUFFER_SIZE);

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(
                false,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ));

        // DMA buffer require a static life-time
        let mut send = tx_buffer;
        let mut receive = rx_buffer;

        send.copy_from_slice(&[0x55u8; 4096]);
        for byte in 0..send.len() {
            send[byte] = byte as u8;
        }

        let transfer = spi.dma_transfer(&mut send, &mut receive).unwrap();
        transfer.wait().unwrap();
        assert_eq!(send, receive);
    }
}

//! SPI Full Duplex DMA Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO8
//!
//! Connect MISO (GPIO2) and MOSI (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
};

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;
    use embedded_hal::spi::SpiBus;
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        spi::master::dma::SpiDmaBus,
    };

    use super::*;

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer() {
        const DMA_BUFFER_SIZE: usize = 4;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    fn test_asymmetric_dma_transfer() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4, 2);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice()[0..1], dma_rx_buf.as_slice()[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer_huge_buffer() {
        const DMA_BUFFER_SIZE: usize = 4096;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        for (i, d) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *d = i as _;
        }

        let transfer = spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer() {
        const DMA_BUFFER_SIZE: usize = 4;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let mut spi_bus = SpiDmaBus::new(spi, dma_tx_buf, dma_rx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi_bus.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_asymmetric_transfer() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let mut spi_bus = SpiDmaBus::new(spi, dma_tx_buf, dma_rx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi_bus.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(&tx_buf[0..1], &rx_buf[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer_huge_buffer() {
        const DMA_BUFFER_SIZE: usize = 4096;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(40);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let mut spi_bus = SpiDmaBus::new(spi, dma_tx_buf, dma_rx_buf);

        let tx_buf = core::array::from_fn(|i| i as _);
        let mut rx_buf = [0; DMA_BUFFER_SIZE];

        spi_bus.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }
}

//! SPI Full Duplex DMA Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MOSI    GPIO3
//! MISO    GPIO6
//! CS      GPIO8
//!
//! PCNT    GPIO2
//! OUTPUT  GPIO5 (helper to keep MISO LOW)
//!
//! The idea of using PCNT (input) here is to connect MOSI to it and count the
//! edges of whatever SPI writes (in this test case 3 pos edges).
//!
//! Connect PCNT (GPIO2) and MOSI (GPIO3) and MISO (GPIO6) and GPIO5 pins.

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal_async::spi::SpiBus;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::{Io, Level, Output, Pull},
    pcnt::{
        channel::{EdgeMode, PcntInputConfig, PcntSource},
        Pcnt,
    },
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
};

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[test]
    #[timeout(3)]
    async fn test_async_dma_read_dma_write_pcnt() {
        const DMA_BUFFER_SIZE: usize = 5;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let sclk = io.pins.gpio0;
        let mosi_mirror = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let miso = io.pins.gpio6;
        let cs = io.pins.gpio8;

        let mut out_pin = Output::new(io.pins.gpio5, Level::Low);
        out_pin.set_low();
        assert_eq!(out_pin.is_set_low(), true);

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure_for_async(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let receive = rx_buffer;

        // Fill the buffer where each byte has 3 pos edges.
        tx_buffer.fill(0b0110_1010);

        assert_eq!(out_pin.is_set_low(), true);

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBus::read(&mut spi, receive).await.unwrap();
            assert_eq!(receive, &[0, 0, 0, 0, 0]);

            SpiBus::write(&mut spi, tx_buffer).await.unwrap();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    async fn test_async_dma_read_dma_transfer_pcnt() {
        const DMA_BUFFER_SIZE: usize = 5;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let sclk = io.pins.gpio0;
        let mosi_mirror = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let miso = io.pins.gpio6;
        let cs = io.pins.gpio8;

        let mut out_pin = Output::new(io.pins.gpio5, Level::High);
        out_pin.set_low();
        assert_eq!(out_pin.is_set_low(), true);

        let dma = Dma::new(peripherals.DMA);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        let dma_channel = dma.spi2channel;
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(
                dma_channel.configure_for_async(false, DmaPriority::Priority0),
                tx_descriptors,
                rx_descriptors,
            );

        let unit = pcnt.unit0;
        unit.channel0.set_edge_signal(PcntSource::from_pin(
            mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let receive = rx_buffer;

        // Fill the buffer where each byte has 3 pos edges.
        tx_buffer.fill(0b0110_1010);

        assert_eq!(out_pin.is_set_low(), true);

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBus::read(&mut spi, receive).await.unwrap();
            assert_eq!(receive, &[0, 0, 0, 0, 0]);

            SpiBus::transfer(&mut spi, receive, tx_buffer)
                .await
                .unwrap();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }
}

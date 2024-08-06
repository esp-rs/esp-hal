//! SPI Full Duplex Async Test
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

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use esp_hal::{
        clock::ClockControl,
        dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
        dma_buffers,
        gpio::Io,
        peripherals::Peripherals,
        prelude::*,
        spi::{
            master::{dma::asynch::SpiDmaAsyncBus, prelude::*, Spi},
            SpiMode,
        },
        system::SystemControl,
    };

    #[init]
    async fn init() {}

    #[test]
    #[timeout(3)]
    async fn test_transfer() {
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
            .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0));

        let mut spi_bus = SpiDmaAsyncBus::new(spi, dma_tx_buf, dma_rx_buf);

        let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
        let mut buffer = [0; 8];

        embedded_hal_async::spi::SpiBus::transfer(&mut spi_bus, &mut buffer, &send_buffer)
            .await
            .unwrap();

        assert_eq!(send_buffer, buffer);
    }
}

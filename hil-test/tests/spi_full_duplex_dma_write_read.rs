//! SPI Full Duplex DMA write + read Test
//! See issue #2059
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2 / GPIO9  (esp32s2 / esp32s3) / GPIO26 (esp32)
//! GPIO    GPIO3 / GPIO10 (esp32s2 / esp32s3) / GPIO27 (esp32)
//!
//! Connect MISO and GPIO pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::Io,
    peripherals::SPI2,
    prelude::*,
    spi::{
        master::{Spi, SpiDma},
        FullDuplexMode,
        SpiMode,
    },
    Blocking,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "esp32",
        feature = "esp32s2",
    ))] {
        use esp_hal::dma::Spi2DmaChannel as DmaChannel0;
    } else {
        use esp_hal::dma::DmaChannel0;
    }
}

struct Context {
    spi: SpiDma<'static, SPI2, DmaChannel0, FullDuplexMode, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;
    use esp_hal::gpio::{Level, Output};

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let (miso, gpio) = hil_test::common_test_pins!(io);
        let _gpio = Output::new(gpio.degrade(), Level::High);

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
            .with_sck(sclk)
            .with_mosi(esp_hal::gpio::DummyPin::new())
            .with_miso(miso)
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        Context { spi }
    }

    #[test]
    #[timeout(3)]
    fn test_write_read(ctx: Context) {
        let spi = ctx.spi;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
        let (spi, dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
        let (spi, mut dma_rx_buf) = transfer.wait();

        let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
        let (spi, _dma_tx_buf) = transfer.wait();

        dma_rx_buf.as_mut_slice().fill(0);
        let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
        let (_, dma_rx_buf) = transfer.wait();

        assert_eq!(&[0xff, 0xff, 0xff, 0xff], dma_rx_buf.as_slice());
    }
}

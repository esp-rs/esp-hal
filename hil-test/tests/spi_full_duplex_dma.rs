//! SPI Full Duplex DMA ASYNC Test
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MISO    GPIO2
//! MOSI    GPIO3
//! CS      GPIO8
//!
//! Connect MISO (GPIO2) and MOSI (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::Io,
    peripherals::{Peripherals, SPI2},
    prelude::*,
    spi::{
        master::{Spi, SpiDma},
        FullDuplexMode,
        SpiMode,
    },
    system::SystemControl,
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
    use esp_hal::dma::{DmaRxBuf, DmaTxBuf};

    use super::*;

    #[init]
    fn init() -> Context {
        let System {
            peripherals,
            clocks,
            ..
        } = esp_hal::init(CpuClock::boot_default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let mosi = io.pins.gpio3;
        let miso = io.pins.gpio2;
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        Context { spi }
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer(ctx: Context) {
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = ctx
            .spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    fn test_asymmetric_dma_transfer(ctx: Context) {
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4, 2);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        dma_tx_buf.fill(&[0xde, 0xad, 0xbe, 0xef]);

        let transfer = ctx
            .spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice()[0..1], dma_rx_buf.as_slice()[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_symmetric_dma_transfer_huge_buffer(ctx: Context) {
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4096);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        for (i, d) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *d = i as _;
        }

        let transfer = ctx
            .spi
            .dma_transfer(dma_tx_buf, dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let (_, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
        assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer(ctx: Context) {
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = ctx.spi.with_buffers(dma_tx_buf, dma_rx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_asymmetric_transfer(ctx: Context) {
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = ctx.spi.with_buffers(dma_tx_buf, dma_rx_buf);

        let tx_buf = [0xde, 0xad, 0xbe, 0xef];
        let mut rx_buf = [0; 4];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(&tx_buf[0..1], &rx_buf[0..1]);
    }

    #[test]
    #[timeout(3)]
    fn test_dma_bus_symmetric_transfer_huge_buffer(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4096;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(40);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = ctx.spi.with_buffers(dma_tx_buf, dma_rx_buf);

        let tx_buf = core::array::from_fn(|i| i as _);
        let mut rx_buf = [0; DMA_BUFFER_SIZE];

        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(tx_buf, rx_buf);
    }
}

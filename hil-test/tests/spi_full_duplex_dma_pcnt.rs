//! SPI Full Duplex DMA ASYNC Test with PCNT readback.

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{interconnect::InputSignal, Io},
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
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
    pcnt_source: InputSignal,
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let (_, mosi) = hil_test::common_test_pins!(io);

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let mosi_loopback = mosi.peripheral_input();
        let mosi_loopback_pcnt = mosi.peripheral_input();
        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_miso(mosi_loopback)
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let pcnt = Pcnt::new(peripherals.PCNT);

        Context {
            spi,
            pcnt_source: mosi_loopback_pcnt,
            pcnt_unit: pcnt.unit0,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_tx_buf) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    fn test_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi
                .dma_transfer(dma_rx_buf, dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }
}

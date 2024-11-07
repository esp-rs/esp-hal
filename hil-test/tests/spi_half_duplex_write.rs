//! SPI Half Duplex Write Test

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{interconnect::InputSignal, Io},
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
    prelude::*,
    spi::{
        master::{Address, Command, Config, Spi, SpiDma},
        SpiDataMode,
        SpiMode,
    },
    Blocking,
};
use hil_test as _;

struct Context {
    spi: SpiDma<'static, Blocking>,
    pcnt_unit: Unit<'static, 0>,
    pcnt_source: InputSignal,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.IO_MUX);
        let sclk = peripherals.pins.gpio0;
        let (mosi, _) = hil_test::common_test_pins!(peripherals);

        let pcnt = Pcnt::new(peripherals.PCNT);
        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let (mosi_loopback, mosi) = mosi.split();

        let spi = Spi::new_with_config(
            peripherals.SPI2,
            Config {
                frequency: 100.kHz(),
                mode: SpiMode::Mode0,
                ..Config::default()
            },
        )
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            pcnt_source: mosi_loopback,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_are_correctly_by_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);

        let transfer = spi
            .half_duplex_write(
                SpiDataMode::Single,
                Command::None,
                Address::None,
                0,
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_tx_buf) = transfer.wait();

        assert_eq!(unit.get_value(), (3 * DMA_BUFFER_SIZE) as _);

        let transfer = spi
            .half_duplex_write(
                SpiDataMode::Single,
                Command::None,
                Address::None,
                0,
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait();

        assert_eq!(unit.get_value(), (6 * DMA_BUFFER_SIZE) as _);
    }

    #[test]
    #[timeout(3)]
    fn test_spidmabus_writes_are_correctly_by_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (rx, rxd, buffer, descriptors) = dma_buffers!(1, DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rxd, rx).unwrap();
        let dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_buffers(dma_rx_buf, dma_tx_buf);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let buffer = [0b0110_1010; DMA_BUFFER_SIZE];
        // Write the buffer where each byte has 3 pos edges.
        spi.half_duplex_write(
            SpiDataMode::Single,
            Command::None,
            Address::None,
            0,
            &buffer,
        )
        .unwrap();

        assert_eq!(unit.get_value(), (3 * DMA_BUFFER_SIZE) as _);

        spi.half_duplex_write(
            SpiDataMode::Single,
            Command::None,
            Address::None,
            0,
            &buffer,
        )
        .unwrap();

        assert_eq!(unit.get_value(), (6 * DMA_BUFFER_SIZE) as _);
    }
}

//! SPI Half Duplex Read Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
    spi::{
        master::{Address, Command, Config, Spi, SpiDma},
        DataMode,
        Mode,
    },
    time::RateExtU32,
    Blocking,
};
use hil_test as _;

struct Context {
    spi: SpiDma<'static, Blocking>,
    miso_mirror: Output<'static>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sclk = peripherals.GPIO0;
        let (miso, miso_mirror) = hil_test::common_test_pins!(peripherals);

        let miso_mirror =
            Output::new(miso_mirror, OutputConfig::default().with_level(Level::High)).unwrap();

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(100.kHz())
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_miso(miso)
        .with_dma(dma_channel);

        Context { spi, miso_mirror }
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (buffer, descriptors, _, _) = dma_buffers!(DMA_BUFFER_SIZE, 0);
        let mut dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();

        // SPI should read '0's from the MISO pin
        ctx.miso_mirror.set_low();

        let mut spi = ctx.spi;

        let transfer = spi
            .half_duplex_read(
                DataMode::Single,
                Command::None,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_rx_buf) = transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), &[0x00; DMA_BUFFER_SIZE]);

        // SPI should read '1's from the MISO pin
        ctx.miso_mirror.set_high();

        let transfer = spi
            .half_duplex_read(
                DataMode::Single,
                Command::None,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();

        (_, dma_rx_buf) = transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), &[0xFF; DMA_BUFFER_SIZE]);
    }

    #[test]
    fn test_spidmabus_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        // WAS THIS AN ERROR?
        let (buffer, descriptors, tx, txd) = dma_buffers!(DMA_BUFFER_SIZE, 1);
        let dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(txd, tx).unwrap();

        let mut spi = ctx.spi.with_buffers(dma_rx_buf, dma_tx_buf);

        // SPI should read '0's from the MISO pin
        ctx.miso_mirror.set_low();

        let mut buffer = [0xAA; DMA_BUFFER_SIZE];
        spi.half_duplex_read(
            DataMode::Single,
            Command::None,
            Address::None,
            0,
            &mut buffer,
        )
        .unwrap();

        assert_eq!(buffer.as_slice(), &[0x00; DMA_BUFFER_SIZE]);

        // SPI should read '1's from the MISO pin
        ctx.miso_mirror.set_high();

        spi.half_duplex_read(
            DataMode::Single,
            Command::None,
            Address::None,
            0,
            &mut buffer,
        )
        .unwrap();

        assert_eq!(buffer.as_slice(), &[0xFF; DMA_BUFFER_SIZE]);
    }
}

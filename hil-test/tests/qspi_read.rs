//! QSPI Read Test

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaPriority, DmaRxBuf},
    dma_buffers,
    gpio::{AnyPin, Level, NoPin, Output},
    prelude::*,
    spi::{
        master::{Address, Command, Spi, SpiDma},
        HalfDuplexMode,
        SpiDataMode,
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
    spi: esp_hal::peripherals::SPI2,
    dma_channel: Channel<'static, DmaChannel0, Blocking>,
    miso: AnyPin,
    miso_mirror: Output<'static>,
}

fn execute(
    mut spi: SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    mut miso_mirror: Output<'static>,
    wanted: u8,
) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
    let mut dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();

    miso_mirror.set_low();

    let transfer = spi
        .read(
            SpiDataMode::Quad,
            Command::None,
            Address::None,
            0,
            dma_rx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_rx_buf) = transfer.wait();

    assert_eq!(dma_rx_buf.as_slice(), &[0; DMA_BUFFER_SIZE]);

    // SPI should read all '1's
    miso_mirror.set_high();

    let transfer = spi
        .read(
            SpiDataMode::Quad,
            Command::None,
            Address::None,
            0,
            dma_rx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();

    (_, dma_rx_buf) = transfer.wait();

    assert_eq!(dma_rx_buf.as_slice(), &[wanted; DMA_BUFFER_SIZE]);
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = peripherals.GPIO.pins();

        let (miso, miso_mirror) = hil_test::common_test_pins!(io);

        let miso = miso.degrade();
        let miso_mirror = Output::new(miso_mirror, Level::High);

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let dma_channel = dma_channel.configure(false, DmaPriority::Priority0);

        Context {
            spi: peripherals.SPI2,
            dma_channel,
            miso,
            miso_mirror,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_0(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, ctx.miso, NoPin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute(spi, ctx.miso_mirror, 0b0001_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_1(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, ctx.miso, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute(spi, ctx.miso_mirror, 0b0010_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_2(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, ctx.miso, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        // SPI should read '0b10111011'
        super::execute(spi, ctx.miso_mirror, 0b0100_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_3(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, NoPin, ctx.miso, NoPin)
            .with_dma(ctx.dma_channel);

        // SPI should read '0b01110111'
        super::execute(spi, ctx.miso_mirror, 0b1000_1000);
    }
}

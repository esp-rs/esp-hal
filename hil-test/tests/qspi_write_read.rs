//! QSPI Write + Read Test
//!
//! Make sure issue #1860 doesn't affect us

//% CHIPS: esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Io, Level, Output},
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
    mosi: AnyPin,
    mosi_mirror: Output<'static>,
}

fn execute(
    mut spi: SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    mut mosi_mirror: Output<'static>,
    wanted: u8,
) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (rx_buffer, rx_descriptors, buffer, descriptors) =
        dma_buffers!(DMA_BUFFER_SIZE, DMA_BUFFER_SIZE);
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

    dma_tx_buf.fill(&[0xff; DMA_BUFFER_SIZE]);

    let transfer = spi
        .write(
            SpiDataMode::Quad,
            Command::Command8(wanted as u16, SpiDataMode::Quad),
            Address::Address24(
                wanted as u32 | (wanted as u32) << 8 | (wanted as u32) << 16,
                SpiDataMode::Quad,
            ),
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, _) = transfer.wait();

    mosi_mirror.set_low();

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

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (mosi, mosi_mirror) = hil_test::common_test_pins!(io);

        let mosi = mosi.degrade();
        let mosi_mirror = Output::new(mosi_mirror, Level::High);

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
            mosi,
            mosi_mirror,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_0(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(
                esp_hal::gpio::NO_PIN,
                Some(ctx.mosi),
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
            )
            .with_dma(ctx.dma_channel);

        super::execute(spi, ctx.mosi_mirror, !0b0001_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                Some(ctx.mosi),
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
            )
            .with_dma(ctx.dma_channel);

        super::execute(spi, ctx.mosi_mirror, !0b0010_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                Some(ctx.mosi),
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
            )
            .with_dma(ctx.dma_channel);

        super::execute(spi, ctx.mosi_mirror, !0b0100_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                esp_hal::gpio::NO_PIN,
                Some(ctx.mosi),
                esp_hal::gpio::NO_PIN,
            )
            .with_dma(ctx.dma_channel);

        super::execute(spi, ctx.mosi_mirror, !0b1000_1000);
    }
}

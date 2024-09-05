//! QSPI Write Test
//!
//! This uses PCNT to count the edges of the MOSI signal
//!
//! Following pins are used:
//! MOSI    GPIO2 / GPIO9 (esp32s2 and esp32s3)
//!
//! PCNT    GPIO3 / GPIO10 (esp32s2 and esp32s3)
//!
//! Connect MOSI and PCNT pins.

//% CHIPS: esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaPriority, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Io, Pull},
    pcnt::{
        channel::{EdgeMode, PcntInputConfig, PcntSource},
        unit::Unit,
        Pcnt,
    },
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
    pcnt: esp_hal::peripherals::PCNT,
    dma_channel: Channel<'static, DmaChannel0, Blocking>,
    mosi: AnyPin<'static>,
    mosi_mirror: AnyPin<'static>,
}

fn execute(
    unit: Unit<'static, 0>,
    mut spi: SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    write: u8,
) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (buffer, descriptors, _, _) = dma_buffers!(DMA_BUFFER_SIZE, 0);
    let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

    dma_tx_buf.fill(&[write; DMA_BUFFER_SIZE]);

    let transfer = spi
        .write(
            SpiDataMode::Quad,
            Command::Command8(write as u16, SpiDataMode::Quad),
            Address::Address24(
                write as u32 | (write as u32) << 8 | (write as u32) << 16,
                SpiDataMode::Quad,
            ),
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    (spi, dma_tx_buf) = transfer.wait();

    assert_eq!(unit.get_value(), 8);

    dma_tx_buf.set_length(0);
    let transfer = spi
        .write(
            SpiDataMode::Quad,
            Command::Command8(write as u16, SpiDataMode::Quad),
            Address::Address24(
                write as u32 | (write as u32) << 8 | (write as u32) << 16,
                SpiDataMode::Quad,
            ),
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    _ = transfer.wait();

    assert_eq!(unit.get_value(), 8 + 4);
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

        let mosi = AnyPin::new(mosi);
        let mosi_mirror = AnyPin::new(mosi_mirror);

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
            pcnt: peripherals.PCNT,
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

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::None },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_0001);
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

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::None },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_0010);
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

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::None },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_0100);
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

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::None },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_1000);
    }
}

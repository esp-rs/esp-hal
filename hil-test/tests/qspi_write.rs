//! QSPI Write Test

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaPriority, DmaTxBuf},
    dma_buffers,
    gpio::{interconnect::InputSignal, AnyPin, Io, NoPin},
    pcnt::{channel::EdgeMode, unit::Unit, Pcnt},
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
    if #[cfg(any(esp32, esp32s2))] {
        use esp_hal::dma::Spi2DmaChannel as DmaChannel0;
    } else {
        use esp_hal::dma::DmaChannel0;
    }
}

struct Context {
    spi: esp_hal::peripherals::SPI2,
    pcnt_source: InputSignal,
    pcnt: esp_hal::peripherals::PCNT,
    dma_channel: Channel<'static, DmaChannel0, Blocking>,
    mosi: AnyPin,
}

fn execute(
    unit: Unit<'static, 0>,
    mut spi: SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    write: u8,
) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
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

        let (mosi, _) = hil_test::common_test_pins!(io);

        let mosi = mosi.degrade();

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let dma_channel = dma_channel.configure(false, DmaPriority::Priority0);

        Context {
            spi: peripherals.SPI2,
            pcnt_source: mosi.peripheral_input(),
            pcnt: peripherals.PCNT,
            dma_channel,
            mosi,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_0(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, ctx.mosi, NoPin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, ctx.mosi, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, ctx.mosi, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, NoPin, ctx.mosi, NoPin)
            .with_dma(ctx.dma_channel);

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        super::execute(unit, spi, 0b0000_1000);
    }
}

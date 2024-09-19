//! QSPI Write Test

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaPriority, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Io, NoPin},
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
    pcnt: esp_hal::peripherals::PCNT,
    dma_channel: Channel<'static, DmaChannel0, Blocking>,
    gpios: [AnyPin; 2],
}

fn transfer(
    spi: SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    dma_tx_buf: DmaTxBuf,
    write: u8,
    command_data_mode: SpiDataMode,
) -> (
    SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    DmaTxBuf,
) {
    let transfer = spi
        .write(
            SpiDataMode::Quad,
            Command::Command8(write as u16, command_data_mode),
            Address::Address24(
                write as u32 | (write as u32) << 8 | (write as u32) << 16,
                SpiDataMode::Quad,
            ),
            0,
            dma_tx_buf,
        )
        .map_err(|e| e.0)
        .unwrap();
    transfer.wait()
}

fn execute(
    unit: Unit<'static, 0>,
    mut spi: SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    write: u8,
) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
    let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            let modes = [SpiDataMode::Single];
        } else {
            let modes = [SpiDataMode::Single, SpiDataMode::Quad];
        }
    }
    for command_data_mode in modes {
        dma_tx_buf.fill(&[write; DMA_BUFFER_SIZE]);

        // Send command + data.
        // Should read 8 bits: 1 command bit, 3 address bits, 4 data bits
        unit.clear();
        (spi, dma_tx_buf) = transfer(spi, dma_tx_buf, write, command_data_mode);
        assert_eq!(unit.get_value(), 8);

        // Send command + address only
        // Should read 4 bits: 1 command bit, 3 address bits
        dma_tx_buf.set_length(0);
        unit.clear();
        (spi, dma_tx_buf) = transfer(spi, dma_tx_buf, write, command_data_mode);
        assert_eq!(unit.get_value(), 4);
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        // MOSI and channel 0 counts data bits on the first line (single-bit transfers,
        // some 4-bit transfers)
        // GPIO counts bits in 4-bit transfers that are not on the first line
        let (mosi, gpio) = hil_test::unconnected_test_pins!(io);

        let mosi = mosi.degrade();
        let gpio = gpio.degrade();

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
            pcnt: peripherals.PCNT,
            dma_channel,
            gpios: [mosi, gpio],
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_0(ctx: Context) {
        let [mosi, _] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(mosi.peripheral_input());
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, mosi, NoPin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute(unit, spi, 0b0000_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        let [mosi, gpio] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(mosi.peripheral_input());
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit.channel1.set_edge_signal(gpio.peripheral_input());
        unit.channel1
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, mosi, gpio, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute(unit, spi, 0b0000_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        let [mosi, gpio] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(mosi.peripheral_input());
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit.channel1.set_edge_signal(gpio.peripheral_input());
        unit.channel1
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, mosi, NoPin, gpio, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute(unit, spi, 0b0000_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        let [mosi, gpio] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(mosi.peripheral_input());
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit.channel1.set_edge_signal(gpio.peripheral_input());
        unit.channel1
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, mosi, NoPin, NoPin, gpio, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute(unit, spi, 0b0000_1000);
    }
}

//! QSPI Test Suite

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: defmt

#![no_std]
#![no_main]

#[cfg(pcnt)]
use esp_hal::pcnt::{channel::EdgeMode, unit::Unit, Pcnt};
use esp_hal::{
    dma::{Channel, Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Io, Level, NoPin, Output},
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

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        const COMMAND_DATA_MODES: [SpiDataMode; 1] = [SpiDataMode::Single];
    } else {
        const COMMAND_DATA_MODES: [SpiDataMode; 2] = [SpiDataMode::Single, SpiDataMode::Quad];
    }
}

type SpiUnderTest =
    SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>;

struct Context {
    spi: esp_hal::peripherals::SPI2,
    #[cfg(pcnt)]
    pcnt: esp_hal::peripherals::PCNT,
    dma_channel: Channel<'static, DmaChannel0, Blocking>,
    gpios: [AnyPin; 3],
}

fn transfer_read(
    spi: SpiUnderTest,
    dma_rx_buf: DmaRxBuf,
    command: Command,
) -> (SpiUnderTest, DmaRxBuf) {
    let transfer = spi
        .read(SpiDataMode::Quad, command, Address::None, 0, dma_rx_buf)
        .map_err(|e| e.0)
        .unwrap();
    transfer.wait()
}

fn transfer_write(
    spi: SpiUnderTest,
    dma_tx_buf: DmaTxBuf,
    write: u8,
    command_data_mode: SpiDataMode,
) -> (SpiUnderTest, DmaTxBuf) {
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

fn execute_read(mut spi: SpiUnderTest, mut miso_mirror: Output<'static>, expected: u8) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
    let mut dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();

    miso_mirror.set_low();
    (spi, dma_rx_buf) = transfer_read(spi, dma_rx_buf, Command::None);
    assert_eq!(dma_rx_buf.as_slice(), &[0; DMA_BUFFER_SIZE]);

    // Set two bits in the written bytes to 1
    miso_mirror.set_high();
    (_, dma_rx_buf) = transfer_read(spi, dma_rx_buf, Command::None);
    assert_eq!(dma_rx_buf.as_slice(), &[expected; DMA_BUFFER_SIZE]);
}

// Regression test for https://github.com/esp-rs/esp-hal/issues/1860
fn execute_write_read(mut spi: SpiUnderTest, mut mosi_mirror: Output<'static>, expected: u8) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (rx_buffer, rx_descriptors, buffer, descriptors) =
        dma_buffers!(DMA_BUFFER_SIZE, DMA_BUFFER_SIZE);
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

    dma_tx_buf.fill(&[0x00; DMA_BUFFER_SIZE]);

    for command_data_mode in COMMAND_DATA_MODES {
        (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, expected, command_data_mode);

        mosi_mirror.set_high();

        (spi, dma_rx_buf) = transfer_read(
            spi,
            dma_rx_buf,
            Command::Command8(expected as u16, command_data_mode),
        );
        assert_eq!(dma_rx_buf.as_slice(), &[expected; DMA_BUFFER_SIZE]);
    }
}

#[cfg(pcnt)]
fn execute_write(unit: Unit<'static, 0>, mut spi: SpiUnderTest, write: u8) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
    let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

    for command_data_mode in COMMAND_DATA_MODES {
        dma_tx_buf.fill(&[write; DMA_BUFFER_SIZE]);

        // Send command + data.
        // Should read 8 bits: 1 command bit, 3 address bits, 4 data bits
        unit.clear();
        (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
        assert_eq!(unit.get_value(), 8);

        // Send command + address only
        // Should read 4 bits: 1 command bit, 3 address bits
        dma_tx_buf.set_length(0);
        unit.clear();
        (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
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

        let (pin, pin_mirror) = hil_test::common_test_pins!(io);
        let unconnected_pin = hil_test::unconnected_pin!(io);

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
            #[cfg(pcnt)]
            pcnt: peripherals.PCNT,
            dma_channel,
            gpios: [
                pin.degrade(),
                pin_mirror.degrade(),
                unconnected_pin.degrade(),
            ],
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, pin, NoPin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, pin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, pin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, NoPin, pin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, pin, NoPin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, pin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, pin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, NoPin, NoPin, NoPin, pin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_0(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [_, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit = pcnt.unit0;

        unit.channel0.set_edge_signal(mosi.peripheral_input());
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = Spi::new_half_duplex(ctx.spi, 100.kHz(), SpiMode::Mode0)
            .with_pins(NoPin, mosi, NoPin, NoPin, NoPin, NoPin)
            .with_dma(ctx.dma_channel);

        super::execute_write(unit, spi, 0b0000_0001);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

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

        super::execute_write(unit, spi, 0b0000_0010);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

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

        super::execute_write(unit, spi, 0b0000_0100);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

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

        super::execute_write(unit, spi, 0b0000_1000);
    }
}

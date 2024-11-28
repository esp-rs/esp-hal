//! QSPI Test Suite

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

#[cfg(pcnt)]
use esp_hal::pcnt::{channel::EdgeMode, unit::Unit, Pcnt};
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Input, Level, Output, Pull},
    prelude::*,
    spi::{
        master::{Address, Command, Config, Spi, SpiDma},
        SpiDataMode,
        SpiMode,
    },
    Blocking,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(pdma)] {
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

type SpiUnderTest = SpiDma<'static, Blocking>;

struct Context {
    spi: Spi<'static, Blocking>,
    #[cfg(pcnt)]
    pcnt: esp_hal::peripherals::PCNT,
    dma_channel: DmaChannel0,
    gpios: [AnyPin; 3],
}

fn transfer_read(
    spi: SpiUnderTest,
    dma_rx_buf: DmaRxBuf,
    command: Command,
) -> (SpiUnderTest, DmaRxBuf) {
    let transfer = spi
        .half_duplex_read(SpiDataMode::Quad, command, Address::None, 0, dma_rx_buf)
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
        .half_duplex_write(
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
fn execute_write(
    unit0: Unit<'static, 0>,
    unit1: Unit<'static, 1>,
    mut spi: SpiUnderTest,
    write: u8,
    data_on_multiple_pins: bool,
) {
    const DMA_BUFFER_SIZE: usize = 4;

    let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
    let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

    for command_data_mode in COMMAND_DATA_MODES {
        dma_tx_buf.fill(&[write; DMA_BUFFER_SIZE]);

        // Send command + address + data.
        // Should read 8 high bits: 1 command bit, 3 address bits, 4 data bits
        unit0.clear();
        unit1.clear();
        (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
        assert_eq!(unit0.value() + unit1.value(), 8);

        if data_on_multiple_pins {
            if command_data_mode == SpiDataMode::Single {
                assert_eq!(unit0.value(), 1);
                assert_eq!(unit1.value(), 7);
            } else {
                assert_eq!(unit0.value(), 0);
                assert_eq!(unit1.value(), 8);
            }
        }

        // Send command + address only
        // Should read 4 bits high: 1 command bit, 3 address bits
        dma_tx_buf.set_length(0);
        unit0.clear();
        unit1.clear();
        (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
        assert_eq!(unit0.value() + unit1.value(), 4);

        if data_on_multiple_pins {
            if command_data_mode == SpiDataMode::Single {
                assert_eq!(unit0.value(), 1);
                assert_eq!(unit1.value(), 3);
            } else {
                assert_eq!(unit0.value(), 0);
                assert_eq!(unit1.value(), 4);
            }
        }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mut pin, mut pin_mirror) = hil_test::common_test_pins!(peripherals);
        let mut unconnected_pin = hil_test::unconnected_pin!(peripherals);

        // Make sure pins have no pullups
        let _ = Input::new(&mut pin, Pull::Down);
        let _ = Input::new(&mut pin_mirror, Pull::Down);
        let _ = Input::new(&mut unconnected_pin, Pull::Down);

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
                .with_mode(SpiMode::Mode0),
        )
        .unwrap();

        Context {
            spi,
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

        let spi = ctx.spi.with_mosi(pin).with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_miso(pin).with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_sio2(pin).with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_reads_correctly_from_gpio_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_sio3(pin).with_dma(ctx.dma_channel);

        super::execute_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_mosi(pin).with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_miso(pin).with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_sio2(pin).with_dma(ctx.dma_channel);

        super::execute_write_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_and_reads_correctly_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High);

        let spi = ctx.spi.with_sio3(pin).with_dma(ctx.dma_channel);

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
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = mosi.split();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx.spi.with_mosi(mosi).with_dma(ctx.dma_channel);

        super::execute_write(unit0, unit1, spi, 0b0000_0001, false);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = mosi.split();
        let (gpio_loopback, gpio) = gpio.split();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(gpio_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx
            .spi
            .with_mosi(mosi)
            .with_miso(gpio)
            .with_dma(ctx.dma_channel);

        super::execute_write(unit0, unit1, spi, 0b0000_0010, true);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = mosi.split();
        let (gpio_loopback, gpio) = gpio.split();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(gpio_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx
            .spi
            .with_mosi(mosi)
            .with_sio2(gpio)
            .with_dma(ctx.dma_channel);

        super::execute_write(unit0, unit1, spi, 0b0000_0100, true);
    }

    #[test]
    #[timeout(3)]
    #[cfg(pcnt)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = mosi.split();
        let (gpio_loopback, gpio) = gpio.split();

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(gpio_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx
            .spi
            .with_mosi(mosi)
            .with_sio3(gpio)
            .with_dma(ctx.dma_channel);

        super::execute_write(unit0, unit1, spi, 0b0000_1000, true);
    }
}

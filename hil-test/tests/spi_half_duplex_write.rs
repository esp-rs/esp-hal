//! SPI Half Duplex Write Test
//!
//! Following pins are used:
//! SCLK    GPIO0
//! MOSI    GPIO2
//!
//! PCNT    GPIO3
//!
//! Connect MOSI (GPIO2) and PCNT (GPIO3) pins.

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority, DmaTxBuf},
    dma_buffers,
    gpio::{GpioPin, Io, Pull},
    pcnt::{
        channel::{EdgeMode, PcntInputConfig, PcntSource},
        unit::Unit,
        Pcnt,
    },
    peripherals::{Peripherals, SPI2},
    prelude::*,
    spi::{
        master::{dma::SpiDma, Address, Command, Spi},
        HalfDuplexMode,
        SpiDataMode,
        SpiMode,
    },
    system::SystemControl,
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
    spi: SpiDma<'static, SPI2, DmaChannel0, HalfDuplexMode, Blocking>,
    pcnt_unit: Unit<'static, 0>,
    mosi_mirror: GpioPin<3>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let mosi = io.pins.gpio2;
        let mosi_mirror = io.pins.gpio3;

        let pcnt = Pcnt::new(peripherals.PCNT);
        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        Context {
            spi,
            mosi_mirror,
            pcnt_unit: pcnt.unit0,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_spi_writes_are_correctly_by_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (buffer, descriptors, _, _) = dma_buffers!(DMA_BUFFER_SIZE, 0);
        let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);

        let transfer = spi
            .write(
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
            .write(
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
}

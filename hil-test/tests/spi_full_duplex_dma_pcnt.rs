//! SPI Full Duplex DMA ASYNC Test with PCNT readback.
//!
//! Folowing pins are used:
//! SCLK    GPIO0
//! MOSI    GPIO3
//! CS      GPIO8
//! PCNT    GPIO2
//! OUTPUT  GPIO5 (helper to keep MISO LOW)
//!
//! The idea of using PCNT (input) here is to connect MOSI to it and count the
//! edges of whatever SPI writes (in this test case 3 pos edges).
//!
//! Connect MISO (GPIO2) and MOSI (GPIO3) pins.

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{GpioPin, Io, Level, Output, Pull},
    pcnt::{
        channel::{EdgeMode, PcntInputConfig, PcntSource},
        unit::Unit,
        Pcnt,
    },
    peripherals::SPI2,
    prelude::*,
    spi::{
        master::{Spi, SpiDma},
        FullDuplexMode,
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
    spi: SpiDma<'static, SPI2, DmaChannel0, FullDuplexMode, Blocking>,
    pcnt_unit: Unit<'static, 0>,
    out_pin: Output<'static, GpioPin<5>>,
    mosi_mirror: GpioPin<2>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let (peripherals, clocks) = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let sclk = io.pins.gpio0;
        let mosi = io.pins.gpio3;
        let miso = unsafe { hil_test::SPI_MISO_Pin::steal() };
        let cs = io.pins.gpio8;

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

        let pcnt = Pcnt::new(peripherals.PCNT);

        let mut out_pin = Output::new(io.pins.gpio5, Level::Low);
        out_pin.set_low();
        assert_eq!(out_pin.is_set_low(), true);
        let mosi_mirror = io.pins.gpio2;

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            out_pin,
            mosi_mirror,
        }
    }

    #[test]
    #[timeout(3)]
    fn test_dma_read_dma_write_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        assert_eq!(ctx.out_pin.is_set_low(), true);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi.dma_write(dma_tx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_tx_buf) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    fn test_dma_read_dma_transfer_pcnt(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 5;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.as_mut_slice().fill(0b0110_1010);

        assert_eq!(ctx.out_pin.is_set_low(), true);

        for i in 1..4 {
            dma_rx_buf.as_mut_slice().copy_from_slice(&[5, 5, 5, 5, 5]);
            let transfer = spi.dma_read(dma_rx_buf).map_err(|e| e.0).unwrap();
            (spi, dma_rx_buf) = transfer.wait();
            assert_eq!(dma_rx_buf.as_slice(), &[0, 0, 0, 0, 0]);

            let transfer = spi
                .dma_transfer(dma_tx_buf, dma_rx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (spi, (dma_tx_buf, dma_rx_buf)) = transfer.wait();
            assert_eq!(unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }
}

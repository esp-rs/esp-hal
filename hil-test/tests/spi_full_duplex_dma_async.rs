//! SPI Full Duplex DMA Test
//!
//! Following pins are used:
//! SCLK    GPIO0
//! MOSI    GPIO3
//! MISO    GPIO6
//! CS      GPIO8
//!
//! PCNT    GPIO2
//! OUTPUT  GPIO5 (helper to keep MISO LOW)
//!
//! The idea of using PCNT (input) here is to connect MOSI to it and count the
//! edges of whatever SPI writes (in this test case 3 pos edges).
//!
//! Connect PCNT (GPIO2) and MOSI (GPIO3) and MISO (GPIO6) and GPIO5 pins.

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embedded_hal_async::spi::SpiBus;
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
        master::{Spi, SpiDmaBus},
        FullDuplexMode,
        SpiMode,
    },
    Async,
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

const DMA_BUFFER_SIZE: usize = 5;

struct Context {
    spi: SpiDmaBus<'static, SPI2, DmaChannel0, FullDuplexMode, Async>,
    pcnt_unit: Unit<'static, 0>,
    out_pin: Output<'static, GpioPin<5>>,
    mosi_mirror: GpioPin<2>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let (peripherals, clocks) = esp_hal::init(Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let sclk = io.pins.gpio0;
        let mosi_mirror = io.pins.gpio2;
        let mosi = io.pins.gpio3;
        let miso = io.pins.gpio6;
        let cs = io.pins.gpio8;

        let mut out_pin = Output::new(io.pins.gpio5, Level::Low);
        out_pin.set_low();
        assert_eq!(out_pin.is_set_low(), true);

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
            .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            out_pin,
            mosi_mirror,
        }
    }

    #[test]
    #[timeout(3)]
    async fn test_async_dma_read_dma_write_pcnt(mut ctx: Context) {
        ctx.pcnt_unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = [0; DMA_BUFFER_SIZE];

        // Fill the buffer where each byte has 3 pos edges.
        let transmit = [0b0110_1010; DMA_BUFFER_SIZE];

        assert_eq!(ctx.out_pin.is_set_low(), true);

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBus::read(&mut ctx.spi, &mut receive).await.unwrap();
            assert_eq!(receive, [0, 0, 0, 0, 0]);

            SpiBus::write(&mut ctx.spi, &transmit).await.unwrap();
            assert_eq!(ctx.pcnt_unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }

    #[test]
    #[timeout(3)]
    async fn test_async_dma_read_dma_transfer_pcnt(mut ctx: Context) {
        ctx.pcnt_unit.channel0.set_edge_signal(PcntSource::from_pin(
            ctx.mosi_mirror,
            PcntInputConfig { pull: Pull::Down },
        ));
        ctx.pcnt_unit
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let mut receive = [0; DMA_BUFFER_SIZE];

        // Fill the buffer where each byte has 3 pos edges.
        let transmit = [0b0110_1010; DMA_BUFFER_SIZE];

        assert_eq!(ctx.out_pin.is_set_low(), true);

        for i in 1..4 {
            receive.copy_from_slice(&[5, 5, 5, 5, 5]);
            SpiBus::read(&mut ctx.spi, &mut receive).await.unwrap();
            assert_eq!(receive, [0, 0, 0, 0, 0]);

            SpiBus::transfer(&mut ctx.spi, &mut receive, &transmit)
                .await
                .unwrap();
            assert_eq!(ctx.pcnt_unit.get_value(), (i * 3 * DMA_BUFFER_SIZE) as _);
        }
    }
}

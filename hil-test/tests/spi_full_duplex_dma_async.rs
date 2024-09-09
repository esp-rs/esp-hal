//! SPI Full Duplex DMA Test

//% CHIPS: esp32 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embedded_hal_async::spi::SpiBus;
use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{ErasedPin, Io, Level, Output, Pull},
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
    pcnt_source: PcntSource,
    pcnt_unit: Unit<'static, 0>,
    out_pin: Output<'static>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let sclk = io.pins.gpio0;

        let (_, mosi) = hil_test::common_test_pins!(io);
        let miso = io.pins.gpio4;

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

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mosi_loopback = mosi.peripheral_input();
        let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
            .with_sck(sclk)
            .with_mosi(mosi)
            .with_miso(miso)
            .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
            .with_buffers(dma_rx_buf, dma_tx_buf);

        Context {
            spi,
            pcnt_source: PcntSource::from(
                mosi_loopback,
                PcntInputConfig { pull: Pull::Down },
            ),
            pcnt_unit: pcnt.unit0,
            out_pin,
        }
    }

    #[test]
    #[timeout(3)]
    async fn test_async_dma_read_dma_write_pcnt(mut ctx: Context) {
        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
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
        ctx.pcnt_unit.channel0.set_edge_signal(ctx.pcnt_source);
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

//! DMA Mem2Mem Tests

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    dma::{AnyGdmaChannel, BurstConfig, DmaChannelConvert, DmaError, ExternalBurstConfig, Mem2Mem},
    dma_buffers,
    dma_buffers_chunk_size,
    dma_descriptors,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

cfg_if::cfg_if! {
    if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
        type DmaPeripheralType<'d> = esp_hal::peripherals::MEM2MEM1<'d>;
    } else {
        type DmaPeripheralType<'d> = esp_hal::peripherals::SPI2<'d>;
    }
}

struct Context {
    channel: AnyGdmaChannel<'static>,
    dma_peripheral: DmaPeripheralType<'static>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let dma_channel = peripherals.DMA_CH0;

        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
                let dma_peripheral = peripherals.MEM2MEM1;
            } else {
                let dma_peripheral = peripherals.SPI2;
            }
        }

        Context {
            channel: dma_channel.degrade(),
            dma_peripheral,
        }
    }

    #[test]
    fn test_internal_mem2mem(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);

        let mut mem2mem = Mem2Mem::new(ctx.channel, ctx.dma_peripheral)
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
            .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..core::mem::size_of_val(tx_buffer) {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_internal_mem2mem_chunk_size(ctx: Context) {
        const CHUNK_SIZE: usize = 2048;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            dma_buffers_chunk_size!(DATA_SIZE, CHUNK_SIZE);

        let mut mem2mem = Mem2Mem::new(ctx.channel, ctx.dma_peripheral)
            .with_descriptors(
                rx_descriptors,
                tx_descriptors,
                BurstConfig {
                    external_memory: ExternalBurstConfig::Size64,
                    internal_memory: Default::default(),
                },
            )
            .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..core::mem::size_of_val(tx_buffer) {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_tx(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(1024, 0);
        match Mem2Mem::new(ctx.channel, ctx.dma_peripheral).with_descriptors(
            rx_descriptors,
            tx_descriptors,
            Default::default(),
        ) {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_rx(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(0, 1024);
        match Mem2Mem::new(ctx.channel, ctx.dma_peripheral).with_descriptors(
            rx_descriptors,
            tx_descriptors,
            Default::default(),
        ) {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }
}

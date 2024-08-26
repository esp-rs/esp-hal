//! DMA Mem2Mem Tests

//% CHIPS: esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaChannel0, DmaError, DmaPriority, Mem2Mem},
    dma_buffers,
    dma_buffers_chunk_size,
    dma_descriptors,
    prelude::*,
    Blocking,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

cfg_if::cfg_if! {
    if #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))] {
        type DmaPeripheralType = esp_hal::peripherals::SPI2;
    } else {
        type DmaPeripheralType = esp_hal::peripherals::MEM2MEM1;
    }
}

struct Context {
    channel: Channel<'static, DmaChannel0, Blocking>,
    dma_peripheral: DmaPeripheralType,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let (peripherals, clocks) = esp_hal::init(Config::default());

        let dma = Dma::new(peripherals.DMA);
        let channel = dma.channel0.configure(false, DmaPriority::Priority0);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))] {
                let dma_peripheral = peripherals.SPI2;
            } else {
                let dma_peripheral = peripherals.MEM2MEM1;
            }
        }

        Context {
            channel,
            dma_peripheral,
        }
    }

    #[test]
    fn test_internal_mem2mem(ctx: Context) {
        let (tx_buffer, tx_descriptors, mut rx_buffer, rx_descriptors) = dma_buffers!(DATA_SIZE);

        let mut mem2mem = Mem2Mem::new(
            ctx.channel,
            ctx.dma_peripheral,
            tx_descriptors,
            rx_descriptors,
        )
        .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(&tx_buffer, &mut rx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..core::mem::size_of_val(tx_buffer) {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_internal_mem2mem_chunk_size(ctx: Context) {
        const CHUNK_SIZE: usize = 2048;

        let (tx_buffer, tx_descriptors, mut rx_buffer, rx_descriptors) =
            dma_buffers_chunk_size!(DATA_SIZE, CHUNK_SIZE);

        let mut mem2mem = Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            tx_descriptors,
            rx_descriptors,
            CHUNK_SIZE,
        )
        .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(&tx_buffer, &mut rx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..core::mem::size_of_val(tx_buffer) {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_tx(ctx: Context) {
        use esp_hal::dma::CHUNK_SIZE;

        let (tx_descriptors, rx_descriptors) = dma_descriptors!(0, 1024);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            tx_descriptors,
            rx_descriptors,
            CHUNK_SIZE,
        ) {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_rx(ctx: Context) {
        use esp_hal::dma::CHUNK_SIZE;

        let (tx_descriptors, rx_descriptors) = dma_descriptors!(1024, 0);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            tx_descriptors,
            rx_descriptors,
            CHUNK_SIZE,
        ) {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_chunk_size_too_small(ctx: Context) {
        let (tx_descriptors, rx_descriptors) = dma_descriptors!(1024, 1024);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            tx_descriptors,
            rx_descriptors,
            0,
        ) {
            Err(DmaError::InvalidChunkSize) => (),
            _ => panic!("Expected InvalidChunkSize"),
        }
    }

    #[test]
    fn test_mem2mem_errors_chunk_size_too_big(ctx: Context) {
        let (tx_descriptors, rx_descriptors) = dma_descriptors!(1024, 1024);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            tx_descriptors,
            rx_descriptors,
            4093,
        ) {
            Err(DmaError::InvalidChunkSize) => (),
            _ => panic!("Expected InvalidChunkSize"),
        }
    }
}

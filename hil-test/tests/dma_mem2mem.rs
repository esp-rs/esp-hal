//! DMA Mem2Mem Tests

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Channel, Dma, DmaError, DmaPriority, Mem2Mem},
    dma_buffers,
    dma_buffers_chunk_size,
    dma_descriptors,
    Blocking,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

cfg_if::cfg_if! {
    if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
        type DmaPeripheralType = esp_hal::peripherals::MEM2MEM1;
    } else {
        type DmaPeripheralType = esp_hal::peripherals::SPI2;
    }
}

struct Context {
    channel: Channel<'static, Blocking>,
    dma_peripheral: DmaPeripheralType,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
                let dma_peripheral = peripherals.MEM2MEM1;
            } else {
                let dma_peripheral = peripherals.SPI2;
            }
        }

        Context {
            channel: dma_channel
                .configure(false, DmaPriority::Priority0)
                .degrade(),
            dma_peripheral,
        }
    }

    #[test]
    fn test_internal_mem2mem(ctx: Context) {
        let (mut rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);

        let mut mem2mem = Mem2Mem::new(
            ctx.channel,
            ctx.dma_peripheral,
            rx_descriptors,
            tx_descriptors,
        )
        .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(&mut rx_buffer, &tx_buffer).unwrap();
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
            rx_descriptors,
            tx_descriptors,
            CHUNK_SIZE,
        )
        .unwrap();

        for i in 0..core::mem::size_of_val(tx_buffer) {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(&mut rx_buffer, &tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..core::mem::size_of_val(tx_buffer) {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_tx(ctx: Context) {
        use esp_hal::dma::CHUNK_SIZE;

        let (rx_descriptors, tx_descriptors) = dma_descriptors!(1024, 0);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            rx_descriptors,
            tx_descriptors,
            CHUNK_SIZE,
        ) {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_rx(ctx: Context) {
        use esp_hal::dma::CHUNK_SIZE;

        let (rx_descriptors, tx_descriptors) = dma_descriptors!(0, 1024);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            rx_descriptors,
            tx_descriptors,
            CHUNK_SIZE,
        ) {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_chunk_size_too_small(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(1024, 1024);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            rx_descriptors,
            tx_descriptors,
            0,
        ) {
            Err(DmaError::InvalidChunkSize) => (),
            _ => panic!("Expected InvalidChunkSize"),
        }
    }

    #[test]
    fn test_mem2mem_errors_chunk_size_too_big(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(1024, 1024);
        match Mem2Mem::new_with_chunk_size(
            ctx.channel,
            ctx.dma_peripheral,
            rx_descriptors,
            tx_descriptors,
            4093,
        ) {
            Err(DmaError::InvalidChunkSize) => (),
            _ => panic!("Expected InvalidChunkSize"),
        }
    }
}

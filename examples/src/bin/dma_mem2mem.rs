//! Uses DMA to copy memory to memory.

//% FEATURES: esp-hal/log
//% CHIPS: esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf, Mem2Mem},
    dma_buffers,
    prelude::*,
};
use log::{error, info};

const DATA_SIZE: usize = 1024 * 10;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let dma = Dma::new(peripherals.DMA);
    let channel = dma.channel0.configure(false, DmaPriority::Priority0);
    #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))]
    let dma_peripheral = peripherals.SPI2;
    #[cfg(not(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3")))]
    let dma_peripheral = peripherals.MEM2MEM1;

    let mem2mem = Mem2Mem::new(channel, dma_peripheral);

    for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
        *b = (i % 256) as u8;
    }

    info!("Starting transfer of {} bytes", DATA_SIZE);

    let tx_transfer = mem2mem.tx.send(dma_tx_buf, true).map_err(|e| e.0).unwrap();
    let rx_transfer = mem2mem
        .rx
        .receive(dma_rx_buf, true)
        .map_err(|e| e.0)
        .unwrap();

    info!("Transfer started");

    (_, _, dma_tx_buf) = tx_transfer.wait();
    (_, dma_rx_buf) = rx_transfer.stop();

    info!("Transfer completed, comparing buffer");

    let mut error = false;
    for (i, (rx, tx)) in dma_rx_buf
        .as_slice()
        .iter()
        .zip(dma_tx_buf.as_slice())
        .enumerate()
    {
        if rx != tx {
            error!(
                "Error: tx_buffer[{}] = {}, rx_buffer[{}] = {}",
                i, tx, i, rx
            );
            error = true;
            break;
        }
    }
    if !error {
        info!("Buffers are equal");
    }
    info!("Done");

    loop {
        delay.delay(2.secs());
    }
}

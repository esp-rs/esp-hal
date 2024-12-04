//! Uses DMA to copy memory to memory.

//% FEATURES: esp-hal/log esp-hal/unstable
//% CHIPS: esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, dma::Mem2Mem, dma_buffers, prelude::*};
use log::{error, info};

const DATA_SIZE: usize = 1024 * 10;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    let (mut rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))] {
            let dma_peripheral = peripherals.SPI2;
        } else {
            let dma_peripheral = peripherals.MEM2MEM1;
        }
    }

    let mut mem2mem = Mem2Mem::new(
        peripherals.DMA_CH0,
        dma_peripheral,
        rx_descriptors,
        tx_descriptors,
    )
    .unwrap();

    for i in 0..core::mem::size_of_val(tx_buffer) {
        tx_buffer[i] = (i % 256) as u8;
    }

    info!("Starting transfer of {} bytes", DATA_SIZE);
    let result = mem2mem.start_transfer(&mut rx_buffer, tx_buffer);
    match result {
        Ok(dma_wait) => {
            info!("Transfer started");
            dma_wait.wait().unwrap();
            info!("Transfer completed, comparing buffer");
            let mut error = false;
            for i in 0..core::mem::size_of_val(tx_buffer) {
                if rx_buffer[i] != tx_buffer[i] {
                    error!(
                        "Error: tx_buffer[{}] = {}, rx_buffer[{}] = {}",
                        i, tx_buffer[i], i, rx_buffer[i]
                    );
                    error = true;
                    break;
                }
            }
            if !error {
                info!("Buffers are equal");
            }
            info!("Done");
        }
        Err(e) => {
            error!("start_transfer: Error: {:?}", e);
        }
    }

    loop {
        delay.delay(2.secs());
    }
}

//! Uses DMA to copy memory to memory.

//% FEATURES: esp-hal/unstable
//% CHIPS: esp32s2 esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{BurstConfig, Mem2Mem},
    dma_buffers,
    main,
    time::Duration,
};
use log::{error, info};

esp_bootloader_esp_idf::esp_app_desc!();

const DATA_SIZE: usize = 1024 * 10;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32s2")] {
            let mem2mem = Mem2Mem::new(peripherals.DMA_COPY);
        } else if #[cfg(any(feature = "esp32c2", feature = "esp32c3", feature = "esp32s3"))] {
            let mem2mem = Mem2Mem::new(peripherals.DMA_CH0, peripherals.SPI2);
        } else {
            let mem2mem = Mem2Mem::new(peripherals.DMA_CH0, peripherals.MEM2MEM1);
        }
    }

    let mut mem2mem = mem2mem
        .with_descriptors(rx_descriptors, tx_descriptors, BurstConfig::default())
        .unwrap();

    for i in 0..size_of_val(tx_buffer) {
        tx_buffer[i] = (i % 256) as u8;
    }

    info!("Starting transfer of {} bytes", DATA_SIZE);
    let result = mem2mem.start_transfer(rx_buffer, tx_buffer);
    match result {
        Ok(dma_wait) => {
            info!("Transfer started");
            dma_wait.wait().unwrap();
            info!("Transfer completed, comparing buffer");
            let mut error = false;
            for i in 0..size_of_val(tx_buffer) {
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
        delay.delay(Duration::from_secs(2));
    }
}

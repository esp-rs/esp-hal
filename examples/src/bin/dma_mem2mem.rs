//! Uses DMA to copy memory to memory.
//!

//% FEATURES: esp-hal/log
//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPeripheral, DmaPriority, Mem2Mem},
    dma_buffers,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use log::{error, info};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    const DATA_SIZE: usize = 1024 * 100;
    let (tx_buffer, tx_descriptors, mut rx_buffer, rx_descriptors) = dma_buffers!(DATA_SIZE);

    let dma = Dma::new(peripherals.DMA);
    let channel = dma.channel0.configure(false, DmaPriority::Priority0);

    let mut mem2mem = Mem2Mem::new(channel, DmaPeripheral::Adc, tx_descriptors, rx_descriptors);

    for i in 0..core::mem::size_of_val(tx_buffer) {
        tx_buffer[i] = (i % 256) as u8;
    }

    info!("Starting transfer of {} bytes", DATA_SIZE);
    let result = mem2mem.start_transfer(&tx_buffer, &mut rx_buffer);
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

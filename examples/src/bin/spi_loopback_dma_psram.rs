//! SPI loopback test using DMA - send from PSRAM receive to internal RAM
//!
//! The following wiring is assumed:
//! - SCLK => GPIO42
//! - MISO => (looped back to MOSI via the GPIO MUX)
//! - MOSI => GPIO48
//! - CS   => GPIO38
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//!
//! If your module is quad PSRAM then you need to change the `psram` feature in the
//! in the features line below to `quad-psram`.

//% FEATURES: esp-hal/log esp-hal/octal-psram
//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaBufBlkSize, DmaRxBuf, DmaTxBuf},
    peripheral::Peripheral,
    prelude::*,
    spi::{
        master::{Config, Spi},
        SpiMode,
    },
};
extern crate alloc;
use log::*;

macro_rules! dma_alloc_buffer {
    ($size:expr, $align:expr) => {{
        let layout = core::alloc::Layout::from_size_align($size, $align).unwrap();
        unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        }
    }};
}

const DMA_BUFFER_SIZE: usize = 8192;
const DMA_ALIGNMENT: DmaBufBlkSize = DmaBufBlkSize::Size64;
const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Starting SPI loopback test");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    let delay = Delay::new();

    let sclk = peripherals.GPIO42;
    let mosi = peripherals.GPIO48;
    let miso = unsafe { mosi.clone_unchecked() };
    let cs = peripherals.GPIO38;

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (_, tx_descriptors) =
        esp_hal::dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
    let tx_buffer = dma_alloc_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize);
    info!(
        "TX: {:p} len {} ({} descripters)",
        tx_buffer.as_ptr(),
        tx_buffer.len(),
        tx_descriptors.len()
    );
    let mut dma_tx_buf =
        DmaTxBuf::new_with_block_size(tx_descriptors, tx_buffer, Some(DMA_ALIGNMENT)).unwrap();
    let (rx_buffer, rx_descriptors, _, _) = esp_hal::dma_buffers!(DMA_BUFFER_SIZE, 0);
    info!(
        "RX: {:p} len {} ({} descripters)",
        rx_buffer.as_ptr(),
        rx_buffer.len(),
        rx_descriptors.len()
    );
    let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    // Need to set miso first so that mosi can overwrite the
    // output connection (because we are using the same pin to loop back)
    let mut spi = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 100.kHz(),
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    )
    .with_sck(sclk)
    .with_miso(miso)
    .with_mosi(mosi)
    .with_cs(cs)
    .with_dma(dma_channel);

    delay.delay_millis(100); // delay to let the above messages display

    for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
        *v = (i % 256) as u8;
    }

    let mut i = 0;

    loop {
        dma_tx_buf.as_mut_slice()[0] = i;
        *dma_tx_buf.as_mut_slice().last_mut().unwrap() = i;
        i = i.wrapping_add(1);

        let transfer = spi
            .transfer(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();

        (spi, (dma_rx_buf, dma_tx_buf)) = transfer.wait();
        for (i, v) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            if dma_rx_buf.as_slice()[i] != *v {
                error!(
                    "Mismatch at index {}: expected {}, got {}",
                    i,
                    *v,
                    dma_rx_buf.as_slice()[i]
                );
                break;
            }
        }
        info!(
            "{:0x?} .. {:0x?}",
            &dma_rx_buf.as_slice()[..10],
            &dma_rx_buf.as_slice().last_chunk::<10>().unwrap()
        );
        dma_tx_buf.as_mut_slice().reverse();
        delay.delay_millis(1000);
    }
}

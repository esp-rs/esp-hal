//! SPI loopback test using DMA - send from PSRAM receive to internal RAM
//!
//! The following wiring is assumed:
//! - SCLK => GPIO42 (esp32s3) / GPIO6 (esp32s2, esp32c5, c61)
//! - MOSI/MISO => GPIO48 (esp32s3) / GPIO7 (esp32s2, esp32c5, c61)
//! - CS => GPIO38 (esp32s3) / GPIO10 (esp32s2, esp32c5, c61)
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.

//% CHIP_FILTER: dma_can_access_psram

#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::ExternalBurstConfig,
    dma_rx_buffer,
    main,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
};
use log::*;

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! dma_alloc_tx_buffer {
    ($size:expr, $align:expr) => {{
        let layout = core::alloc::Layout::from_size_align($size, $align as usize).unwrap();
        let buffer = unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        };

        const DMA_CHUNK_SIZE: usize = 4096 - $align as usize;
        let descriptors = esp_hal::dma_descriptors_impl!($size, DMA_CHUNK_SIZE);
        esp_hal::dma::DmaTxBuf::new_with_config(
            descriptors,
            unsafe { esp_hal::dma::aligned::DmaAlignedMut::new_unchecked(buffer) },
            $align,
        )
    }};
}

const DMA_BUFFER_SIZE: usize = 8192;
const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size64;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Starting SPI loopback test");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    let delay = Delay::new();

    let (sclk, mosi, cs) = cfg_select! {
        feature = "esp32s3" => (peripherals.GPIO42, peripherals.GPIO48, peripherals.GPIO38),
        _ => (peripherals.GPIO6, peripherals.GPIO7, peripherals.GPIO10),
    };
    let miso = unsafe { mosi.clone_unchecked() };

    let dma_channel = cfg_select! {
        feature = "esp32s2" => peripherals.DMA_SPI2,
        feature = "esp32p4" => peripherals.DMA_AXI_CH0,
        _ => peripherals.DMA_CH0,
    };

    let mut dma_tx_buf = dma_alloc_tx_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT).unwrap();
    let mut dma_rx_buf = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();
    // Need to set miso first so that mosi can overwrite the
    // output connection (because we are using the same pin to loop back)
    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
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
            .transfer_buffers(dma_rx_buf.len(), dma_rx_buf, dma_tx_buf.len(), dma_tx_buf)
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

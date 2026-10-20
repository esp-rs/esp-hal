//! SPI loopback test using DMA - send from PSRAM receive to internal RAM
//!
//! The following wiring is assumed:
//!
//! Signal    | ESP32-S3 | ESP32-S2 | ESP32-C5/C61/P4 | ESP32-S31 |
//! --------- | -------- | -------- | --------------- | --------- |
//! SCLK      | GPIO42   | GPIO6    | GPIO6           | GPIO11    |
//! MOSI/MISO | GPIO48   | GPIO7    | GPIO7           | GPIO12    |
//! CS        | GPIO38   | GPIO10   | GPIO10          | GPIO13    |
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example requires a board with PSRAM. If no PSRAM is detected, the
//! allocation will fail at runtime.
//!
//! This example transfers data via SPI. MISO/MOSI are connected together by the firmware.

//% CHIP_FILTER: dma_can_access_psram

#![no_std]
#![no_main]

extern crate alloc;

use allocator_api2::boxed::Box;
use esp_alloc::DmaCompatibleExternalMemory;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{DmaTxBuf, ExternalBurstConfig, aligned::DmaAlignedMut},
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

const DMA_BUFFER_SIZE: usize = 8192;
const DMA_ALIGNMENT: ExternalBurstConfig = cfg_select! {
    // ExternalBurstConfig::Size64 is not available on
    // ESP32-S2.
    feature = "esp32s2" => ExternalBurstConfig::Size32,
    _ => ExternalBurstConfig::Size64,
};

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("Starting SPI loopback test");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    let delay = Delay::new();

    let (sclk, mosi, cs) =
        cfg_select! {
            feature = "esp32s3" => (peripherals.GPIO42, peripherals.GPIO48, peripherals.GPIO38),
            feature = "esp32s31" => (peripherals.GPIO11, peripherals.GPIO12, peripherals.GPIO13),
            _ => (peripherals.GPIO6, peripherals.GPIO7, peripherals.GPIO10),
        };

    let (miso, mosi) = unsafe { mosi.split() };

    let dma_channel = cfg_select! {
        feature = "esp32s2" => peripherals.DMA_SPI2,
        any(feature = "esp32p4", feature = "esp32s31") => peripherals.DMA_AXI_CH0,
        _ => peripherals.DMA_CH0,
    };

    let buffer = Box::leak(Box::new_in(
        [0u8; DMA_BUFFER_SIZE],
        DmaCompatibleExternalMemory,
    ));

    const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;
    let descriptors = esp_hal::dma_descriptors_impl!(DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
    let mut dma_tx_buf = DmaTxBuf::new_with_config(
        descriptors,
        DmaAlignedMut::new(buffer).unwrap().unsize(),
        DMA_ALIGNMENT,
    )
    .unwrap();
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

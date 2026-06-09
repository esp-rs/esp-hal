//! Uses DMA to copy psram to internal memory.

//% CHIP_FILTER: dma_can_access_psram && dma_supports_mem2mem

#![no_std]
#![no_main]

extern crate alloc;

use aligned::{A64, Aligned};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{BurstConfig, ExternalBurstConfig, Mem2Mem},
    dma_descriptors_chunk_size,
    main,
    time::Duration,
};
use log::{error, info};

esp_bootloader_esp_idf::esp_app_desc!();

const DATA_SIZE: usize = 1024 * 10;
const CHUNK_SIZE: usize = 4032; // size is aligned to 64 bytes

macro_rules! dma_buffer_aligned {
    ($size:expr, $align:ty) => {{
        static mut BUFFER: Aligned<$align, [u8; $size]> = Aligned([0; $size]);
        unsafe { &mut *BUFFER }
    }};
}

macro_rules! dma_alloc_buffer {
    ($size:expr, $align:expr) => {{
        let layout = core::alloc::Layout::from_size_align($size, $align).unwrap();
        unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("make_buffers: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        }
    }};
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    let extram_buffer: &mut [u8] = dma_alloc_buffer!(DATA_SIZE, 64);
    let intram_buffer = dma_buffer_aligned!(DATA_SIZE, A64);
    let (rx_descriptors, tx_descriptors) = dma_descriptors_chunk_size!(DATA_SIZE, CHUNK_SIZE);

    let mem2mem = cfg_select! {
        feature = "esp32s2" => Mem2Mem::new(peripherals.DMA_CRYPTO),
        feature = "esp32s3" => Mem2Mem::new(peripherals.DMA_CH0, peripherals.SPI2),
        feature = "esp32p4" => Mem2Mem::new(peripherals.DMA_AXI_CH0),
        _ => Mem2Mem::new(peripherals.DMA_CH0),
    };

    let mut mem2mem = mem2mem
        .with_descriptors(
            rx_descriptors,
            tx_descriptors,
            BurstConfig {
                external_memory: if cfg!(feature = "esp32s2") {
                    ExternalBurstConfig::Size32
                } else {
                    ExternalBurstConfig::Size64
                },
                internal_memory: Default::default(),
            },
        )
        .unwrap();

    for i in 0..core::mem::size_of_val(extram_buffer) {
        extram_buffer[i] = (i % 256) as u8;
        intram_buffer[i] = 255 - extram_buffer[i];
    }

    info!(" ext2int: Starting transfer of {} bytes", DATA_SIZE);
    match mem2mem.start_transfer(intram_buffer, extram_buffer) {
        Ok(dma_wait) => {
            info!("Transfer started");
            dma_wait.wait().unwrap();
            info!("Transfer completed, comparing buffer");
            let mut error = false;
            for i in 0..core::mem::size_of_val(extram_buffer) {
                if intram_buffer[i] != extram_buffer[i] {
                    error!(
                        "Error: extram_buffer[{}] = {}, intram_buffer[{}] = {}",
                        i, extram_buffer[i], i, intram_buffer[i]
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

    for i in 0..core::mem::size_of_val(extram_buffer) {
        intram_buffer[i] = (i % 256) as u8;
        extram_buffer[i] = 255 - intram_buffer[i];
    }

    info!(" int2ext: Starting transfer of {} bytes", DATA_SIZE);
    match mem2mem.start_transfer(extram_buffer, intram_buffer) {
        Ok(dma_wait) => {
            info!("Transfer started");
            dma_wait.wait().unwrap();
            info!("Transfer completed, comparing buffer");
            let mut error = false;
            for i in 0..core::mem::size_of_val(extram_buffer) {
                if intram_buffer[i] != extram_buffer[i] {
                    error!(
                        "Error: extram_buffer[{}] = {}, intram_buffer[{}] = {}",
                        i, extram_buffer[i], i, intram_buffer[i]
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

//! Uses DMA to copy psram to internal memory.
//!
//! If your module is octal PSRAM then you need to set `ESP_HAL_CONFIG_PSRAM_MODE` to `octal`.

//% FEATURES: esp-hal/psram aligned esp-hal/unstable
//% CHIPS: esp32s3

#![no_std]
#![no_main]

use aligned::{Aligned, A64};
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
extern crate alloc;

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

fn init_heap(psram: &esp_hal::peripherals::PSRAM<'_>) {
    let (start, size) = esp_hal::psram::psram_raw_parts(psram);
    info!("init_heap: start: {:p}", start);
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap(&peripherals.PSRAM);

    let delay = Delay::new();

    let extram_buffer: &mut [u8] = dma_alloc_buffer!(DATA_SIZE, 64);
    let intram_buffer = dma_buffer_aligned!(DATA_SIZE, A64);
    let (rx_descriptors, tx_descriptors) = dma_descriptors_chunk_size!(DATA_SIZE, CHUNK_SIZE);

    let dma_peripheral = peripherals.SPI2;

    let mut mem2mem = Mem2Mem::new(peripherals.DMA_CH0, dma_peripheral)
        .with_descriptors(
            rx_descriptors,
            tx_descriptors,
            BurstConfig {
                external_memory: ExternalBurstConfig::Size64,
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

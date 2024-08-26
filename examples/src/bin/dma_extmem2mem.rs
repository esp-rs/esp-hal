//! Uses DMA to copy psram to internal memory.

//% FEATURES: esp-hal/log opsram-2m aligned
//% CHIPS: esp32s3

#![no_std]
#![no_main]

use aligned::{Aligned, A64};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma::{Dma, DmaPriority, Mem2Mem},
    dma_descriptors_chunk_size,
    prelude::*,
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

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap(psram: impl esp_hal::peripheral::Peripheral<P = esp_hal::peripherals::PSRAM>) {
    esp_hal::psram::init_psram(psram);
    info!(
        "init_heap: start: 0x{:0x}",
        esp_hal::psram::psram_vaddr_start()
    );
    unsafe {
        ALLOCATOR.init(
            esp_hal::psram::psram_vaddr_start() as *mut u8,
            esp_hal::psram::PSRAM_BYTES,
        );
    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let (peripherals, clocks) = esp_hal::init(Config::default());

    init_heap(peripherals.PSRAM);
    let delay = Delay::new(&clocks);

    let mut extram_buffer: &mut [u8] = dma_alloc_buffer!(DATA_SIZE, 64);
    let mut intram_buffer = dma_buffer_aligned!(DATA_SIZE, A64);
    let (tx_descriptors, rx_descriptors) = dma_descriptors_chunk_size!(DATA_SIZE, CHUNK_SIZE);

    let dma = Dma::new(peripherals.DMA);
    let channel = dma.channel0.configure(false, DmaPriority::Priority0);
    let dma_peripheral = peripherals.SPI2;

    let mut mem2mem = Mem2Mem::new_with_chunk_size(
        channel,
        dma_peripheral,
        tx_descriptors,
        rx_descriptors,
        CHUNK_SIZE,
    )
    .unwrap();

    for i in 0..core::mem::size_of_val(extram_buffer) {
        extram_buffer[i] = (i % 256) as u8;
        intram_buffer[i] = 255 - extram_buffer[i];
    }

    info!(" ext2int: Starting transfer of {} bytes", DATA_SIZE);
    match mem2mem.start_transfer(&extram_buffer, &mut intram_buffer) {
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
    match mem2mem.start_transfer(&intram_buffer, &mut extram_buffer) {
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
        delay.delay(2.secs());
    }
}

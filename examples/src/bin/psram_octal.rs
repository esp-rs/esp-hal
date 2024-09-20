//! This shows how to use PSRAM as heap-memory via esp-alloc
//!
//! You need an ESP32-S3 with at least 2 MB of PSRAM memory.

//% CHIPS: esp32s3
//% FEATURES: esp-hal/octal-psram

#![no_std]
#![no_main]

extern crate alloc;

use alloc::{string::String, vec::Vec};

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{prelude::*, psram};
use esp_println::println;

fn init_psram_heap(start: *mut u8, size: usize) {
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}

#[cfg(is_not_release)]
compile_error!("PSRAM example must be built in release mode!");

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let (start, size) = psram::init_psram(peripherals.PSRAM, psram::PsramConfig::default());
    init_psram_heap(start, size);

    println!("Going to access PSRAM");
    let mut large_vec: Vec<u32> = Vec::with_capacity(500 * 1024 / 4);

    for i in 0..(500 * 1024 / 4) {
        large_vec.push((i & 0xff) as u32);
    }

    println!("vec size = {} bytes", large_vec.len() * 4);
    println!("vec address = {:p}", large_vec.as_ptr());
    println!("vec[..100] = {:?}", &large_vec[..100]);

    let string = String::from("A string allocated in PSRAM");
    println!("'{}' allocated at {:p}", &string, string.as_ptr());

    println!("done");

    loop {}
}

//! This shows how to use PSRAM as heap-memory via esp-alloc
//!
//! You need an ESP32 with at least 2 MB of PSRAM memory.

#![no_std]
#![no_main]

use esp32_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, psram};
use esp_backtrace as _;
use esp_println::println;

extern crate alloc;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(psram::PSRAM_VADDR_START as *mut u8, psram::PSRAM_BYTES);
    }
}

#[entry]
fn main() -> ! {
    #[cfg(debug_assertions)]
    compile_error!("PSRAM on ESP32 needs to be built in release mode");

    #[cfg(feature = "log")]
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    println!("Going to access PSRAM");

    let mut large_vec: alloc::vec::Vec<u32> = alloc::vec::Vec::with_capacity(500 * 1024 / 4);

    for i in 0..(500 * 1024 / 4) {
        large_vec.push((i & 0xff) as u32);
    }

    println!("vec size = {} bytes", large_vec.len() * 4);
    println!("vec address = {:p}", large_vec.as_ptr());
    println!("{:?}", &large_vec[..100]);

    let string = alloc::string::String::from("A string allocated in PSRAM");
    println!("'{}' allocated at {:p}", &string, string.as_ptr());

    println!("done");

    loop {}
}

//! This shows how to use PSRAM as heap-memory via esp-alloc
//!
//! You need an ESP32-S3 with at least 2 MB of PSRAM memory.

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    soc,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

extern crate alloc;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(
            soc::psram::PSRAM_VADDR_START as *mut u8,
            soc::psram::PSRAM_BYTES,
        );
    }
}

#[entry]
fn main() -> ! {
    #[cfg(debug_assertions)]
    compile_error!("PSRAM on ESP32-S3 needs to be built in release mode");

    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    soc::psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(
        system.clock_control,
        esp_hal_common::clock::CpuClock::Clock240MHz,
    )
    .freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    println!("Going to access PSRAM");
    let mut large_vec: alloc::vec::Vec<u32> = alloc::vec::Vec::with_capacity(500 * 1024 / 4);

    for i in 0..(500 * 1024 / 4) {
        large_vec.push((i & 0xff) as u32);
    }

    println!("vec size = {} bytes", large_vec.len() * 4);
    println!("vec address = {:p}", large_vec.as_ptr());
    println!("vec[..100] = {:?}", &large_vec[..100]);

    let string = alloc::string::String::from("A string allocated in PSRAM");
    println!("'{}' allocated at {:p}", &string, string.as_ptr());

    println!("done");

    loop {}
}

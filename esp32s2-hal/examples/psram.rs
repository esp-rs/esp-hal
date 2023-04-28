//! This shows how to use PSRAM as heap-memory via esp-alloc
//!
//! You need an ESP32-S2 with at least 2 MB of PSRAM memory.

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

extern crate alloc;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    extern "C" {
        static PSRAM_BYTES: usize;
        static PSRAM_VADDR_START: usize;
    }

    unsafe {
        ALLOCATOR.init(PSRAM_VADDR_START as *mut u8, PSRAM_BYTES);
    }
}

#[entry]
fn main() -> ! {
    init_psram_heap();

    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

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

    let string = alloc::string::String::from("A string allocated in PSRAM");
    println!("'{}' allocated at {:p}", &string, string.as_ptr());

    println!("done");

    loop {}
}

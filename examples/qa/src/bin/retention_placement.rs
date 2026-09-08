//! Places a CPU retention buffer where the light-sleep retention DMA reaches it.
//!
//! Build this and read the symbol table to confirm the buffer is inside the range that
//! `sleep.cpu_retention_mem_start` and `_end` give.

//% CHIP_FILTER: esp32s3 || esp32c3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
    ram,
    rtc_cntl::{CpuRetentionMemory, sleep::LowPower},
};

esp_bootloader_esp_idf::esp_app_desc!();

#[ram(reclaimed)]
static mut RETENTION: CpuRetentionMemory = CpuRetentionMemory::new();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut lpwr = LowPower::new(peripherals.LPWR);
    lpwr.install_cpu_retention_memory(unsafe { &mut *(&raw mut RETENTION) })
        .unwrap();

    loop {}
}

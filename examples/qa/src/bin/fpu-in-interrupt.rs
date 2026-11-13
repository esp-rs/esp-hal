//! Uses floating-point code in an interrupt handler without the `float-save-restore` feature.
//!
//! The application must panic with a message that names the `float-save-restore` feature.

//% CHIP_FILTER: esp32h4 || esp32p4 || esp32s31
//% DEFAULT-FEATURES: false

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{interrupt::software::SoftwareInterrupt, main, peripherals::FROM_CPU_INTR2};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut sw_int = SoftwareInterrupt::new(peripherals.FROM_CPU_INTR2);
    sw_int.set_interrupt_handler(sw_int_handler);
    sw_int.raise();

    log::error!("The interrupt handler did not panic");
    loop {}
}

#[esp_hal::handler]
fn sw_int_handler() {
    unsafe { SoftwareInterrupt::new(FROM_CPU_INTR2::steal()) }.reset();
    let value = core::hint::black_box(2.0_f32) * 1.5;
    log::info!("{value}");
}

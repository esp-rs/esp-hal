//! Demonstrates software interrupts

//% CHIP_FILTER: gpio_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    interrupt::software::SoftwareInterrupt,
    main,
    peripherals::FROM_CPU_INTR0,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut sw_int = SoftwareInterrupt::new(peripherals.FROM_CPU_INTR0);

    sw_int.set_interrupt_handler(sw_int_handler);

    let delay = Delay::new();

    loop {
        delay.delay_millis(2500);
        sw_int.raise();
    }
}

#[esp_hal::handler]
fn sw_int_handler() {
    unsafe { SoftwareInterrupt::new(FROM_CPU_INTR0::steal()) }.reset();
    log::info!("Triggered");
}

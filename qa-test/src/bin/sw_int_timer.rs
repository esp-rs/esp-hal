//! Demonstrates software interrupts

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3 esp32c2 esp32p4

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl},
    main,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let mut sw_int = sw_ints.software_interrupt0;

    sw_int.set_interrupt_handler(sw_int_handler);

    let delay = Delay::new();

    loop {
        delay.delay_millis(2500);
        sw_int.raise();
    }
}

#[esp_hal::handler]
fn sw_int_handler() {
    unsafe { SoftwareInterrupt::<'static, 0>::steal() }.reset();
    log::info!("Triggered");
}

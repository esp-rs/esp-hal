//! hello world
//!
//! This is a very basic example that shows how to print messages.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{main, time::Instant};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_log::init_uart!(peripherals);

    log::info!("Init!");

    loop {
        log::info!("Bing!");

        let now = Instant::now();
        while now.elapsed().as_millis() < 5000 {}
    }
}

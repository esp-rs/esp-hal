#![no_std]
#![no_main]

use esp_backtrace as _;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    // nothing really here

    loop {}
}

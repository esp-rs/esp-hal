//% CHIPS: esp32c5

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
    gpio::{Input, InputConfig, Pull}
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    println!("Hello World!");

    let config = InputConfig::default().with_pull(Pull::Down);
    let button = Input::new(peripherals.GPIO4, config);

    loop {
        if button.is_high() {
            println!("O, ABOBA!");
        }
        else {
            println!("OH NO, THEY KILLED ABOBA");
        }
    }
}
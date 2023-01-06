#![no_std]
#![no_main]

use esp32c6_hal::peripherals;
use esp_backtrace as _;

#[riscv_rt::entry]
fn main() -> ! {
    esp_println::println!("Hello!");

    loop {}
}

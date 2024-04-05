//! Demonstrates the use of the hardware Random Number Generator (RNG)

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{peripherals::Peripherals, prelude::*, rng::Trng};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut trng = Trng::new(peripherals.RNG, peripherals.ADC1);

    // Generate a random word (u32):
    println!("Random u32:   {}", trng.random());

    // Fill a buffer with random bytes:
    let mut buf = [0u8; 16];
    trng.read(&mut buf);
    println!("Random bytes: {:?}", buf);

    loop {}
}

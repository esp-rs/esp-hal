//! Demonstrates the use of the hardware Random Number Generator (RNG)

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{peripherals::Peripherals, prelude::*, rng::{Rng, Secure},
    analog::adc::{AdcConfig, Attenuation, ADC},
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
    peripherals::ADC1};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut trng = Rng::<Secure>::new(peripherals.RNG, peripherals.ADC1);

    // let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // cfg_if::cfg_if! {
    //     if #[cfg(feature = "esp32")] {
    //         let analog_pin = io.pins.gpio32.into_analog();
    //     } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
    //         let analog_pin = io.pins.gpio3.into_analog();
    //     } else {
    //         let analog_pin = io.pins.gpio2.into_analog();
    //     }
    // }

    // // Create ADC instances
    // let mut adc1_config = AdcConfig::new();
    // let mut adc1_pin = adc1_config.enable_pin(analog_pin, Attenuation::Attenuation11dB);
    // let mut adc1 = ADC::<ADC1>::new(peripherals.ADC1, adc1_config);


    // Generate a random word (u32):
    println!("Random u32:   {}", trng.random());

    // Fill a buffer with random bytes:
    let mut buf = [0u8; 16];
    trng.read(&mut buf);
    println!("Random bytes: {:?}", buf);

    loop {}
}

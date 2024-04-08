//! Demonstrates the use of the hardware Random Number Generator (RNG)

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{prelude::*, rng::Trng,
    analog::adc::{AdcConfig, Attenuation, ADC},
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
    peripherals::{Peripherals, ADC1}};

use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let analog_pin = io.pins.gpio3.into_analog();
    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin = adc1_config.enable_pin(analog_pin, Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::new(peripherals.ADC1, adc1_config);
    let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
    
    let mut trng = Trng::new(peripherals.RNG, &mut adc1);
    
    let (mut rng, adc1) = trng.downgrade();
    
    // Fill a buffer with random bytes:
    let mut buf = [0u8; 16];
    // trng.read(&mut buf);
    println!("Random bytes: {:?}", buf);

    loop {
        println!("Random u32:   {}", rng.random());
        let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
        println!("ADC reading = {}", pin_value);
    }
}

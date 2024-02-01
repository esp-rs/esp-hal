//! Connect a potentiometer to PIN3 and see the read values change when
//! rotating the shaft. Alternatively you could also connect the PIN to GND or
//! 3V3 to see the maximum and minimum raw values read.

#![no_std]
#![no_main]

use esp32s2_hal::{
    adc::{AdcConfig, Attenuation, ADC},
    clock::ClockControl,
    gpio::IO,
    peripherals::{Peripherals, ADC1},
    prelude::*,
    Delay,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create ADC instances
    let mut adc1_config = AdcConfig::new();
    let mut pin3 =
        adc1_config.enable_pin(io.pins.gpio3.into_analog(), Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::new(peripherals.ADC1, adc1_config);

    let mut delay = Delay::new(&clocks);

    loop {
        let pin3_value: u16 = nb::block!(adc1.read(&mut pin3)).unwrap();
        println!("PIN3 ADC reading = {}", pin3_value);
        delay.delay_ms(1500u32);
    }
}

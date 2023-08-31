//! Connect a potentiometer to PIN25 and see the read values change when
//! rotating the shaft. Alternatively you could also connect the PIN to GND or
//! 3V3 to see the maximum and minimum raw values read.

#![no_std]
#![no_main]

use esp32_hal::{
    adc::{AdcConfig, Attenuation, ADC, ADC2},
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    Delay,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create ADC instances
    let analog = peripherals.SENS.split();

    let mut adc2_config = AdcConfig::new();
    let mut pin25 =
        adc2_config.enable_pin(io.pins.gpio25.into_analog(), Attenuation::Attenuation11dB);
    let mut adc2 = ADC::<ADC2>::adc(analog.adc2, adc2_config).unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        let pin25_value: u16 = nb::block!(adc2.read(&mut pin25)).unwrap();
        println!("PIN25 ADC reading = {}", pin25_value);
        delay.delay_ms(1500u32);
    }
}

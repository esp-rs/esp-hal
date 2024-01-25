//! Connect a potentiometer to PIN2 and see the read values change when
//! rotating the shaft. Alternatively you could also connect the PIN to GND or
//! 3V3 to see the maximum and minimum raw values read.

#![no_std]
#![no_main]

use esp32c2_hal::{
    adc,
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
    // You can try any of the following calibration methods by uncommenting
    // them. Note that only AdcCalLine returns readings in mV; the other two
    // return raw readings in some unspecified scale.
    //
    // type AdcCal = ();
    // type AdcCal = adc::AdcCalBasic<ADC1>;
    type AdcCal = adc::AdcCalLine<ADC1>;

    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin_with_cal::<_, AdcCal>(
        io.pins.gpio2.into_analog(),
        Attenuation::Attenuation11dB,
    );
    let mut adc1 = ADC::<ADC1>::adc(peripherals.ADC1, adc1_config).unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        let pin_mv = nb::block!(adc1.read(&mut pin)).unwrap();
        println!("PIN2 ADC reading = {pin_mv} mV");
        delay.delay_ms(1500u32);
    }
}

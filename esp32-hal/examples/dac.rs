//! This example shows how to use the DAC on PIN 25 and 26
//! You can connect an LED (with a suitable resistor) or check the changing
//! voltage using a voltmeter on those pins.

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    dac::{DAC1, DAC2},
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    Delay,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin25 = io.pins.gpio25.into_analog();
    let pin26 = io.pins.gpio26.into_analog();

    // Create DAC instances
    let mut dac1 = DAC1::new(peripherals.DAC1, pin25);
    let mut dac2 = DAC2::new(peripherals.DAC2, pin26);

    let mut delay = Delay::new(&clocks);

    let mut voltage_dac1: u8 = 200;
    let mut voltage_dac2: u8 = 255;
    loop {
        // Change voltage on the pins using write function
        voltage_dac1 = voltage_dac1.wrapping_add(1);
        dac1.write(voltage_dac1);

        voltage_dac2 = voltage_dac2.wrapping_sub(1);
        dac2.write(voltage_dac2);
        delay.delay_ms(50u32);
    }
}

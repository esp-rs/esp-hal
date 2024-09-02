//! This example shows how to use the DAC
//!
//! You can connect an LED (with a suitable resistor) or check the changing
//! voltage using a voltmeter on those pins.
//!
//! When targeting the ESP32, the pins for `DAC1` and `DAC2` are GPIO25 and
//! GPIO26 respectively; for the ESP32-S2, they are GPIO17 and GPIO18.
//!
//! The following wiring is assumed for ESP32:
//! - DAC1 => GPIO25
//! - DAC2 => GPIO26
//! The following wiring is assumed for ESP32S2:
//! - DAC1 => GPIO17
//! - DAC2 => GPIO18

//% CHIPS: esp32 esp32s2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{analog::dac::Dac, delay::Delay, gpio::Io, prelude::*};

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let dac1_pin = io.pins.gpio25;
            let dac2_pin = io.pins.gpio26;
        } else if #[cfg(feature = "esp32s2")] {
            let dac1_pin = io.pins.gpio17;
            let dac2_pin = io.pins.gpio18;
        }
    }

    // Create DAC instances
    let mut dac1 = Dac::new(peripherals.DAC1, dac1_pin);
    let mut dac2 = Dac::new(peripherals.DAC2, dac2_pin);

    let delay = Delay::new(&clocks);

    let mut voltage_dac1: u8 = 200;
    let mut voltage_dac2: u8 = 255;
    loop {
        // Change voltage on the pins using write function
        voltage_dac1 = voltage_dac1.wrapping_add(1);
        dac1.write(voltage_dac1);

        voltage_dac2 = voltage_dac2.wrapping_sub(1);
        dac2.write(voltage_dac2);
        delay.delay_millis(50);
    }
}

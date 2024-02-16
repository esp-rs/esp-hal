//! RGB LED Demo
//!
//! This example drives an 12-element RGB ring that is connected to GPIO33
//!
//! The LEDs in the ring are transitioning though the HSV color spectrum for
//! - Saturation: 255
//! - Hue: 0 - 255
//! - Value: 255
//!
//! For the 12-element RGB ring to work, building the release version is going
//! to be required.

#![no_std]
#![no_main]

use esp32_hal::{clock::ClockControl, peripherals, prelude::*, rmt::Rmt, Delay, IO};
use esp_backtrace as _;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness,
    gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

#[entry]
fn main() -> ! {
    let peripherals = peripherals::Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Configure RMT peripheral globally
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    // -> We need to use the macro `smartLedBuffer!` with the number of addressed
    // LEDs here to initialize the internal LED pulse buffer to the correct
    // size!
    let rmt_buffer = smartLedBuffer!(12);
    let mut led = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio33, rmt_buffer, &clocks);

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;

    loop {
        // Iterate over the rainbow!
        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            let rgb_color = hsv2rgb(color);

            // Assign new color to all 12 LEDs
            data = [rgb_color; 12];

            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 10))
                .unwrap();
            delay.delay_ms(20u8);
        }
    }
}

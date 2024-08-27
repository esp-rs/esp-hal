//! RGB LED Demo
//!
//! This example drives an SK68XX RGB LED, which is connected to a pin on the
//! official DevKits.
//!
//! The demo will leverage the [`smart_leds`](https://crates.io/crates/smart-leds)
//! crate functionality to circle through the HSV hue color space (with
//! saturation and value both at 255). Additionally, we apply a gamma correction
//! and limit the brightness to 10 (out of 255).
//!
//! The following wiring is assumed for ESP32:
//! - LED => GPIO33
//! The following wiring is assumed for ESP32C3:
//! - LED => GPIO8
//! The following wiring is assumed for ESP32C6, ESP32H2:
//! - LED => GPIO8
//! The following wiring is assumed for ESP32S2:
//! - LED => GPIO18
//! The following wiring is assumed for ESP32S3:
//! - LED => GPIO48

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    rmt::Rmt,
    system::SystemControl,
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness,
    gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Each devkit uses a unique GPIO for the RGB LED, so in order to support
    // all chips we must unfortunately use `#[cfg]`s:
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let led_pin = io.pins.gpio33;
        } else if #[cfg(feature = "esp32c3")] {
            let led_pin = io.pins.gpio8;
        } else if #[cfg(any(feature = "esp32c6", feature = "esp32h2"))] {
            let led_pin = io.pins.gpio8;
        } else if #[cfg(feature = "esp32s2")] {
            let led_pin = io.pins.gpio18;
        } else if #[cfg(feature = "esp32s3")] {
            let led_pin = io.pins.gpio48;
        }
    }

    // Configure RMT peripheral globally
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = 32.MHz();
        } else {
            let freq = 80.MHz();
        }
    }

    let rmt = Rmt::new(peripherals.RMT, freq, &clocks).unwrap();

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer, &clocks);

    let delay = Delay::new(&clocks);

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
            data = [hsv2rgb(color)];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 10))
                .unwrap();
            delay.delay_millis(20);
        }
    }
}

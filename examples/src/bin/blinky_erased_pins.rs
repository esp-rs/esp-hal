//! Blinks 3 LEDs
//!
//! The boot button is treated as an input, and will print a message when pressed.
//! This additionally demonstrates passing GPIO to a function in a generic way.
//!
//! The following wiring is assumed:
//! - LEDs => GPIO2, GPIO4, GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, Input, Io, Level, Output, Pin, Pull},
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set LED GPIOs as an output:
    let led1 = Output::new(io.pins.gpio2.degrade(), Level::Low);
    let led2 = Output::new(io.pins.gpio4.degrade(), Level::Low);
    let led3 = Output::new(io.pins.gpio5.degrade(), Level::Low);

    // Use boot button as an input:
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let button = io.pins.gpio0.degrade();
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let button = io.pins.gpio9.degrade();

    let button = Input::new(button, Pull::Up);

    let mut pins = [led1, led2, led3];

    let delay = Delay::new();

    loop {
        toggle_pins(&mut pins, &button);
        delay.delay_millis(500);
    }
}

fn toggle_pins(leds: &mut [Output<AnyPin>], button: &Input<AnyPin>) {
    for pin in leds.iter_mut() {
        pin.toggle();
    }

    if button.is_low() {
        esp_println::println!("Button pressed");
    }
}

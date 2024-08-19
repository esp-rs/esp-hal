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
    gpio::{AnyInput, AnyOutput, Io, Level, Pull},
    prelude::*,
};

#[entry]
fn main() -> ! {
    let System {
        peripherals,
        clocks,
        ..
    } = esp_hal::init(CpuClock::boot_default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set LED GPIOs as an output:
    let led1 = AnyOutput::new(io.pins.gpio2, Level::Low);
    let led2 = AnyOutput::new(io.pins.gpio4, Level::Low);
    let led3 = AnyOutput::new(io.pins.gpio5, Level::Low);

    // Use boot button as an input:
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let button = io.pins.gpio0;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let button = io.pins.gpio9;

    let button = AnyInput::new(button, Pull::Up);

    let mut pins = [led1, led2, led3];

    let delay = Delay::new(&clocks);

    loop {
        toggle_pins(&mut pins, &button);
        delay.delay_millis(500);
    }
}

fn toggle_pins(leds: &mut [AnyOutput], button: &AnyInput) {
    for pin in leds.iter_mut() {
        pin.toggle();
    }

    if button.is_low() {
        esp_println::println!("Button pressed");
    }
}

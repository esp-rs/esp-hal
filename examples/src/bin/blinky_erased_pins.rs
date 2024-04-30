//! Blinks 3 LEDs
//!
//! This assumes that LEDs are connected to GPIO2, 4 and 5.
//!
//! GPIO1 is treated as an input, and will print a message when pressed. This
//! Additionally demonstrates passing GPIO to a function in a generic way.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{AnyPin, Input, Io, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set LED GPIOs as an output:
    let led1 = io.pins.gpio2.into_push_pull_output();
    let led2 = io.pins.gpio4.into_push_pull_output();
    let led3 = io.pins.gpio5.into_push_pull_output();

    // Use boot button as an input:
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let button = io.pins.gpio0.into_pull_down_input().into();
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let button = io.pins.gpio9.into_pull_down_input().into();

    // You can use `into` or `degrade`:
    let mut pins = [led1.into(), led2.into(), led3.degrade().into()];

    // Initialize the `Delay` peripheral, and use it to toggle the LED state
    // in a loop:
    let delay = Delay::new(&clocks);

    loop {
        toggle_pins(&mut pins, &button);
        delay.delay_millis(500);
    }
}

fn toggle_pins(leds: &mut [AnyPin<Output>], button: &AnyPin<Input>) {
    for pin in leds.iter_mut() {
        pin.toggle();
    }

    if button.is_low() {
        esp_println::println!("Button pressed");
    }
}

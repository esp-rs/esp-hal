//! Blinks 3 LEDs
//!
//! This assumes that LEDs are connected to GPIO8, 9 and 10.
//!
//! GPIO1 is treated as an input, and will print a message when pressed. This
//! Additionally demonstrates passing GPIO to a function in a generic way.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embedded-hal-02

#![no_std]
#![no_main]

use embedded_hal_02::digital::v2::{InputPin, ToggleableOutputPin};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{AnyPin, Input, Output, PullDown, PushPull, IO},
    peripherals::Peripherals,
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set LED GPIOs as an output:
    let led1 = io.pins.gpio8.into_push_pull_output();
    let led2 = io.pins.gpio9.into_push_pull_output();
    let led3 = io.pins.gpio10.into_push_pull_output();

    // Set GPIO0 as an input:
    let button = io.pins.gpio0.into_pull_down_input().into();

    // You can use `into` or `degrade`:
    let mut pins = [led1.into(), led2.into(), led3.degrade().into()];

    // Initialize the `Delay` peripheral, and use it to toggle the LED state
    // in a loop:
    let delay = Delay::new(&clocks);

    loop {
        toggle_pins(&mut pins, &button);
        delay.delay_millis(500u32);
    }
}

fn toggle_pins(leds: &mut [AnyPin<Output<PushPull>>], button: &AnyPin<Input<PullDown>>) {
    for pin in leds.iter_mut() {
        pin.toggle().unwrap();
    }

    if button.is_low().unwrap() {
        esp_println::println!("Button pressed");
    }
}

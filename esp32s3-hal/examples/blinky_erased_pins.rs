//! Blinks an LED
//!
//! This assumes that LEDs are connected to GPIO3, 4 and 5.

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    gpio::{AnyPin, Input, Output, PullDown, PushPull, IO},
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

    // Set LED GPIOs as an output.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led1 = io.pins.gpio3.into_push_pull_output();
    let led2 = io.pins.gpio4.into_push_pull_output();
    let led3 = io.pins.gpio5.into_push_pull_output();

    // Set GPIO9 as an input.
    let button = io.pins.gpio0.into_pull_down_input().into();

    // You can use `into` or `degrade`
    let mut pins = [led1.into(), led2.into(), led3.degrade().into()];

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    loop {
        toggle_pins(&mut pins, &button);
        delay.delay_ms(500u32);
    }
}

fn toggle_pins(leds: &mut [AnyPin<Output<PushPull>>], button: &AnyPin<Input<PullDown>>) {
    for pin in leds.iter_mut() {
        pin.toggle().unwrap();
    }

    if button.is_low().unwrap() {
        esp_println::println!("Button");
    }
}

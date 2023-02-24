//! Blinks an LED
//!
//! This assumes that LEDs are connected to GPIO3, 4 and 5.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    gpio::{AnyPin, Input, Output, PullDown, PushPull, IO},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Set GPIO4 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led1 = io.pins.gpio3.into_push_pull_output();
    let led2 = io.pins.gpio4.into_push_pull_output();
    let led3 = io.pins.gpio5.into_push_pull_output();

    let button = io.pins.gpio9.into_pull_down_input().degrade();

    // you can use `into` or `degrade`
    let mut pins = [led1.into(), led2.into(), led3.degrade()];

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

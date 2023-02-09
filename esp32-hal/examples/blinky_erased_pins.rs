//! Blinks three LEDs
//!
//! This assumes that LEDs are connected to GPIO25, 26 and 27.

#![no_std]
#![no_main]

use esp32_hal::{
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
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led1 = io.pins.gpio25.into_push_pull_output();
    let led2 = io.pins.gpio26.into_push_pull_output();
    let led3 = io.pins.gpio27.into_push_pull_output();

    let button = io.pins.gpio0.into_pull_down_input().degrade();

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

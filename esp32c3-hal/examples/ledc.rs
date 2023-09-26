//! Turns on LED with the option to change LED intensity depending on `duty`
//! value, then fades it. Possible starting values (`u32`) are in range 0..100.
//!
//! This assumes that a LED is connected to the pin assigned to `led`. (GPIO4)

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource,
        LowSpeed,
        LEDC,
    },
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = io.pins.gpio4.into_push_pull_output();

    let mut ledc = LEDC::new(peripherals.LEDC, &clocks);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 24u32.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    channel0.start_duty_fade(0, 100, 2000).expect_err(
        "Fading from 0% to 100%, at 24kHz and 5-bit resolution, over 2 seconds, should fail",
    );

    loop {
        // Set up a breathing LED: fade from off to on over a second, then
        // from on back off over the next second.  Then loop.
        channel0.start_duty_fade(0, 100, 1000).unwrap();
        while channel0.is_duty_fade_running() {}
        channel0.start_duty_fade(100, 0, 1000).unwrap();
        while channel0.is_duty_fade_running() {}
    }
}

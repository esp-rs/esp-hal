//! Turns on LED with the option to change LED intensity depending on `duty`
//! value.
//!
//! This assumes that a LED is connected to the pin assigned to `led`. (GPIO4)

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    gpio::IO,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource,
        LowSpeed,
        LEDC,
    },
    pac::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Serial,
};
use esp_println;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let _timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut _serial0 = Serial::new(peripherals.UART0);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timer
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = io.pins.gpio4.into_push_pull_output();

    esp_println::println!("\nESP32S3 Started\n\n");

    let mut ledc = LEDC::new(&clocks, &mut system.peripheral_clock_control);

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);

    let res = lstimer0
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
            duty_pct: 0.1,
        })
        .unwrap();

    loop {}
}

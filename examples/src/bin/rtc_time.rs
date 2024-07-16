//! Prints time in milliseconds from the RTC Timer

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::Rtc,
    system::SystemControl,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let rtc = Rtc::new(peripherals.LPWR);
    let delay = Delay::new(&clocks);

    loop {
        esp_println::println!("rtc time in milliseconds is {}", rtc.get_time_ms());
        delay.delay_millis(1000);
    }
}

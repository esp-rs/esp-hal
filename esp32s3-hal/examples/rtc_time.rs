//! Prints time in milliseconds from the RTC Timer

#![no_std]
#![no_main]

use esp32s3_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, Rtc};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut delay = Delay::new(&clocks);

    loop {
        esp_println::println!("rtc time in milliseconds is {}", rtc.get_time_ms());
        delay.delay_ms(1000u32);
    }
}

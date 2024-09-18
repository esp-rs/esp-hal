//! Prints time in milliseconds from the RTC Timer

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*, rtc_cntl::Rtc};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::config::Config::default());

    let rtc = Rtc::new(peripherals.LPWR);
    let delay = Delay::new();

    loop {
        esp_println::println!(
            "rtc time in milliseconds is {}",
            rtc.current_time().and_utc().timestamp_millis()
        );
        delay.delay_millis(1000);

        // Set the time to half a second in the past
        let new_time = rtc.current_time() - Duration::from_millis(500);
        esp_println::println!(
            "setting rtc time to {} milliseconds",
            new_time.and_utc().timestamp_millis()
        );
        rtc.set_current_time(new_time);
    }
}

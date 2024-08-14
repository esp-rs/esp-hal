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
        // Set the time to 1 second past the current time after 5 seconds have passed
        if rtc.get_time_ms() > 5_000 {
            let new_time = rtc.get_time_ms() + 1_000;
            esp_println::println!("setting rtc time to {new_time} (milliseconds)");
            if rtc.set_time_ms(new_time).is_err() {
                esp_println::println!("failed to set rtc time due to overflow");
            }
        }
    }
}

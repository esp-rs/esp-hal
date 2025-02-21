//! This shows how to use RTC memory.
//!
//! RTC memory is retained during resets and during most sleep modes.
//!
//! Initialized memory is always re-initialized on startup.
//!
//! Persistent memory is not zeroed after resets that preserve RTC ram. See the
//! documentation for `esp-hal-procmacros` for the full list.
//!
//! Zeroed memory is initialized to zero on startup.
//!
//! We can also run code from RTC memory.

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![allow(static_mut_refs)]
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, ram, rtc_cntl::Rtc, time::Duration};
use esp_println::println;

#[ram(rtc_fast)]
static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];

#[ram(rtc_fast, persistent)]
static mut SOME_PERSISTENT_DATA: [u8; 2] = [0; 2];

#[ram(rtc_fast, zeroed)]
static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    // The RWDT flash boot protection must be enabled, as it is triggered as part of
    // the example.
    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.rwdt.enable();

    println!(
        "IRAM function located at {:p}",
        function_in_ram as *const ()
    );
    unsafe {
        assert_eq!(&[0xaa, 0xbb], &SOME_INITED_DATA);
        assert_eq!(&[0; 8], &SOME_ZEROED_DATA);

        SOME_INITED_DATA[0] = 0xff;
        SOME_ZEROED_DATA[0] = 0xff;

        assert_eq!(&[0xff, 0xbb], &SOME_INITED_DATA);
        assert_eq!(&[0xff, 0, 0, 0, 0, 0, 0, 0], &SOME_ZEROED_DATA);

        if SOME_PERSISTENT_DATA[1] == 0xff {
            SOME_PERSISTENT_DATA[1] = 0;
        }

        println!("Counter {}", SOME_PERSISTENT_DATA[1]);
        SOME_PERSISTENT_DATA[1] += 1;
    }

    println!(
        "RTC_FAST function located at {:p}",
        function_in_rtc_ram as *const ()
    );

    assert_eq!(42, function_in_rtc_ram());

    println!("Restarting in ~ 10 seconds.");
    let mut i = 10;
    loop {
        assert_eq!(42, function_in_rtc_ram());
        function_in_ram(i);
        i = i.saturating_sub(1);
        delay.delay(Duration::from_secs(1));
    }
}

#[ram]
fn function_in_ram(count: u32) {
    println!("{}", count);
}

#[ram(rtc_fast)]
fn function_in_rtc_ram() -> u32 {
    42
}

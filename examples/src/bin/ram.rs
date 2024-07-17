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

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    macros::ram,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::Rtc,
    system::SystemControl,
};
use esp_println::println;

#[ram(rtc_fast)]
static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];

#[ram(rtc_fast, persistent)]
static mut SOME_PERSISTENT_DATA: [u8; 2] = [0; 2];

#[ram(rtc_fast, zeroed)]
static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    // The RWDT flash boot protection must be enabled, as it is triggered as part of
    // the example.
    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.rwdt.enable();

    println!(
        "IRAM function located at {:p}",
        function_in_ram as *const ()
    );
    unsafe {
        println!("SOME_INITED_DATA {:x?}", SOME_INITED_DATA);
        println!("SOME_PERSISTENT_DATA {:x?}", SOME_PERSISTENT_DATA);
        println!("SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA);

        SOME_INITED_DATA[0] = 0xff;
        SOME_ZEROED_DATA[0] = 0xff;

        println!("SOME_INITED_DATA {:x?}", SOME_INITED_DATA);
        println!("SOME_PERSISTENT_DATA {:x?}", SOME_PERSISTENT_DATA);
        println!("SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA);

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
    println!("Result {}", function_in_rtc_ram());

    loop {
        function_in_ram();
        delay.delay(1.secs());
    }
}

#[ram]
fn function_in_ram() {
    println!("Hello world!");
}

#[ram(rtc_fast)]
fn function_in_rtc_ram() -> u32 {
    42
}

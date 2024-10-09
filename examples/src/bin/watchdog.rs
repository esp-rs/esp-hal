//! This demos the watchdog timer.
//!
//! Basically the same as `hello_world` but if you remove the call to
//! `wdt.feed()` the watchdog will reset the system.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-hal/log

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*, timer::timg::TimerGroup, interrupt::Priority};
use esp_println::println;

#[entry]
fn main() -> ! {
    // let peripherals = esp_hal::init(esp_hal::Config::default());
    let peripherals = esp_hal::init(esp_hal::Config{
        watchdog: esp_hal::config::WatchdogConfig{
            swd: false,
            rwdt: esp_hal::config::WatchdogStatus::Disabled,
            timg0: esp_hal::config::WatchdogStatus::Disabled,
            timg1: esp_hal::config::WatchdogStatus::Disabled,
            // timg0: esp_hal::config::WatchdogStatus::Enabled(2.secs()),
            // ..Default::default()
        },
        ..Default::default()
    });

    let delay = Delay::new();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut wdt0 = timg0.wdt;
    wdt0.enable();
    wdt0.set_timeout(2.secs());

    // wdt0.set_stage_action(0, esp_hal::timer::timg::MwdtStageAction::Interrupt).unwrap();
    // wdt0.set_interrupt_handler(interrupt_handler);


    loop {
        // wdt0.feed();
        println!("Hello world!");
        delay.delay(1.secs());
    }
}

#[handler(priority = Priority::min())]
fn interrupt_handler() {
    critical_section::with(|cs| {
        println!("RWDT Interrupt");

       panic!();
    });
}

//! This demos the RTC Watchdog Timer (RWDT).
//!
//! The RWDT is initially configured to trigger an interrupt after a given
//! timeout. Then, upon expiration, the RWDT is restarted and then reconfigured
//! to reset both the main system and the RTC.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    Rtc,
    Rwdt,
};

static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));

    #[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
    interrupt::enable(Interrupt::LP_WDT, Priority::Priority1).unwrap();
    #[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
    interrupt::enable(Interrupt::RTC_CORE, Priority::Priority1).unwrap();

    loop {}
}

fn interrupt_handler() {
    critical_section::with(|cs| {
        esp_println::println!("RWDT Interrupt");

        let mut rwdt = RWDT.borrow_ref_mut(cs);
        let rwdt = rwdt.as_mut().unwrap();
        rwdt.clear_interrupt();

        esp_println::println!("Restarting in 5 seconds...");

        rwdt.start(5000u64.millis());
        rwdt.unlisten();
    });
}

#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
#[interrupt]
fn LP_WDT() {
    interrupt_handler();
}

#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
#[interrupt]
fn RTC_CORE() {
    interrupt_handler();
}

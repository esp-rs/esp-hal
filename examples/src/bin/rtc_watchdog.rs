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
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{Rtc, Rwdt},
};

static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.set_interrupt_handler(interrupt_handler);
    rtc.rwdt.set_timeout(2000.millis());
    rtc.rwdt.listen();

    critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));

    loop {}
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn interrupt_handler() {
    critical_section::with(|cs| {
        esp_println::println!("RWDT Interrupt");

        let mut rwdt = RWDT.borrow_ref_mut(cs);
        let rwdt = rwdt.as_mut().unwrap();
        rwdt.clear_interrupt();

        esp_println::println!("Restarting in 5 seconds...");

        rwdt.set_timeout(5000.millis());
        rwdt.unlisten();
    });
}

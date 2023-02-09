//! This demos the RTC Watchdog Timer (RWDT).
//! The RWDT is initially configured to trigger an interrupt after a given
//! timeout. Then, upon expiration, the RWDT is restarted and then reconfigured
//! to reset both the main system and the RTC.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    Rtc,
    Rwdt,
};
use esp_backtrace as _;

static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timer
    rtc.rwdt.disable();

    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));

    interrupt::enable(
        peripherals::Interrupt::RTC_CORE,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    loop {}
}

#[interrupt]
fn RTC_CORE() {
    critical_section::with(|cs| {
        let mut rwdt = RWDT.borrow_ref_mut(cs);
        let rwdt = rwdt.as_mut().unwrap();
        rwdt.clear_interrupt();

        esp_println::println!("Restarting in 5 seconds...");

        rwdt.start(5000u64.millis());
        rwdt.unlisten();
    });
}

//! This demos a simple monitor for the XTAL frequency, by relying on a special
//! feature of the TIMG0 (Timer Group 0). This feature counts the number of XTAL
//! clock cycles within a given number of RTC_SLOW_CLK cycles.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32h2_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    Rtc,
};
use esp_backtrace as _;

static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    esp_println::println!(
        "{: <10} XTAL frequency: {} MHz",
        "[Expected]",
        clocks.xtal_clock.to_MHz()
    );

    interrupt::enable(
        peripherals::Interrupt::LP_WDT,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    critical_section::with(|cs| {
        RTC.borrow_ref_mut(cs).replace(rtc);
    });

    loop {}
}

#[interrupt]
fn LP_WDT() {
    critical_section::with(|cs| {
        let mut rtc = RTC.borrow(cs).borrow_mut();
        let rtc = rtc.as_mut().unwrap();

        esp_println::println!(
            "{: <10} XTAL frequency: {} MHz",
            "[Monitor]",
            rtc.estimate_xtal_frequency()
        );

        rtc.rwdt.clear_interrupt();
    });
}

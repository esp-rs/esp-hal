//! This demos a simple monitor for the XTAL frequency, by relying on a special
//! feature of the TIMG0 (Timer Group 0). This feature counts the number of XTAL
//! clock cycles within a given number of RTC_SLOW_CLK cycles.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embedded-hal-02

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal_02::watchdog::WatchdogEnable;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    rtc_cntl::Rtc,
};
use esp_println::println;

static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.rwdt.start(2000.millis());
    rtc.rwdt.listen();

    println!(
        "{: <10} XTAL frequency: {} MHz",
        "[Expected]",
        clocks.xtal_clock.to_MHz()
    );

    #[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
    interrupt::enable(Interrupt::LP_WDT, Priority::Priority1).unwrap();
    #[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
    interrupt::enable(Interrupt::RTC_CORE, Priority::Priority1).unwrap();

    critical_section::with(|cs| RTC.borrow_ref_mut(cs).replace(rtc));

    loop {}
}

fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut rtc = RTC.borrow_ref_mut(cs);
        let rtc = rtc.as_mut().unwrap();

        println!(
            "{: <10} XTAL frequency: {} MHz",
            "[Monitor]",
            rtc.estimate_xtal_frequency()
        );

        rtc.rwdt.clear_interrupt();
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

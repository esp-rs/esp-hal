//! This demos the RTC Watchdog Timer (RWDT).
//! The RWDT is initially configured to trigger an interrupt after a given
//! timeout. Then, upon expiration, the RWDT is restarted and then reconfigured
//! to reset both the main system and the RTC.

#![no_std]
#![no_main]

use core::cell::RefCell;

use esp32s3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    Rtc,
    Rwdt,
};
use panic_halt as _;
use xtensa_lx::mutex::{CriticalSectionMutex, Mutex};
use xtensa_lx_rt::entry;

static mut RWDT: CriticalSectionMutex<RefCell<Option<Rwdt>>> =
    CriticalSectionMutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();

    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    interrupt::enable(pac::Interrupt::RTC_CORE, interrupt::Priority::Priority1).unwrap();

    unsafe {
        (&RWDT).lock(|data| (*data).replace(Some(rtc.rwdt)));
    }

    loop {}
}

#[interrupt]
fn RTC_CORE() {
    unsafe {
        (&RWDT).lock(|data| {
            esp_println::println!("RWDT Interrupt");

            let mut rwdt = data.borrow_mut();
            let rwdt = rwdt.as_mut().unwrap();

            rwdt.clear_interrupt();

            esp_println::println!("Restarting in 5 seconds...");

            rwdt.start(5000u64.millis());
            rwdt.unlisten();
        });
    }
}

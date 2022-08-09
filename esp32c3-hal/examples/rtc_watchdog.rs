//! This demos the RTC Watchdog Timer (RWDT).
//! The RWDT is initially configured to trigger an interrupt after a given
//! timeout. Then, upon expiration, the RWDT is restarted and then reconfigured
//! to reset both the main system and the RTC.

#![no_std]
#![no_main]

use core::cell::RefCell;

use bare_metal::Mutex;
use esp32c3_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    Rtc,
    Rwdt,
};
use panic_halt as _;
use riscv_rt::entry;

static mut RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

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

    riscv::interrupt::free(|_cs| unsafe {
        RWDT.get_mut().replace(Some(rtc.rwdt));
    });

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn RTC_CORE() {
    riscv::interrupt::free(|cs| unsafe {
        esp_println::println!("RWDT Interrupt");

        let mut rwdt = RWDT.borrow(*cs).borrow_mut();
        let rwdt = rwdt.as_mut().unwrap();

        rwdt.clear_interrupt();

        esp_println::println!("Restarting in 5 seconds...");

        rwdt.start(5000u64.millis());
        rwdt.unlisten();
    });
}

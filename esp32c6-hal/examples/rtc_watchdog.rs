//! This demos the RTC Watchdog Timer (RWDT).
//! The RWDT is initially configured to trigger an interrupt after a given
//! timeout. Then, upon expiration, the RWDT is restarted and then reconfigured
//! to reset both the main system and the RTC.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32c6_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    riscv,
    timer::TimerGroup,
    Rtc,
    Rwdt,
};
use esp_backtrace as _;

static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    wdt0.disable();
    wdt1.disable();
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();

    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    interrupt::enable(
        peripherals::Interrupt::LP_WDT,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn LP_WDT() {
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

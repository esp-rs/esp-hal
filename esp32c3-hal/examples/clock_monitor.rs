//! This demos a simple monitor for the XTAL frequency, by relying on a special feature of the
//! TIMG0 (Timer Group 0). This feature counts the number of XTAL clock cycles within a given
//! number of RTC_SLOW_CLK cycles.

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
};
use panic_halt as _;
use riscv_rt::entry;

static mut RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();

    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    esp_println::println!(
        "{: <10} XTAL frequency: {} MHz",
        "[Expected]",
        clocks.xtal_clock.to_MHz()
    );

    interrupt::enable(pac::Interrupt::RTC_CORE, interrupt::Priority::Priority1).unwrap();

    riscv::interrupt::free(|_cs| unsafe {
        RTC.get_mut().replace(Some(rtc));
    });

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn RTC_CORE() {
    riscv::interrupt::free(|cs| unsafe {
        let mut rtc = RTC.borrow(*cs).borrow_mut();
        let rtc = rtc.as_mut().unwrap();

        esp_println::println!(
            "{: <10} XTAL frequency: {} MHz",
            "[Monitor]",
            rtc.estimate_xtal_frequency()
        );

        rtc.rwdt.clear_interrupt();
    });
}

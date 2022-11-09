//! This demos a simple monitor for the XTAL frequency, by relying on a special
//! feature of the TIMG0 (Timer Group 0). This feature counts the number of XTAL
//! clock cycles within a given number of RTC_SLOW_CLK cycles.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32s2_hal::{
    clock::ClockControl,
    interrupt,
    pac::{self, Peripherals},
    prelude::*,
    Rtc,
};
use esp_backtrace as _;
use xtensa_atomic_emulation_trap as _;
use esp_println::println;
use xtensa_lx_rt::entry;

static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timer
    rtc.rwdt.disable();

    rtc.rwdt.start(2000u64.millis());
    rtc.rwdt.listen();

    println!(
        "{: <10} XTAL frequency: {} MHz",
        "[Expected]",
        clocks.xtal_clock.to_MHz()
    );

    interrupt::enable(pac::Interrupt::RTC_CORE, interrupt::Priority::Priority1).unwrap();

    critical_section::with(|cs| RTC.borrow_ref_mut(cs).replace(rtc));

    loop {}
}

#[interrupt]
fn RTC_CORE() {
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

#[xtensa_lx_rt::exception]
fn exception(cause: xtensa_lx_rt::exception::ExceptionCause, frame: xtensa_lx_rt::exception::Context) {
    use esp_println::*;

    println!("\n\nException occured {:?} {:x?}", cause, frame);
    
    let backtrace = esp_backtrace::arch::backtrace();
    for b in backtrace.iter() {
        if let Some(addr) = b {
            println!("0x{:x}", addr)
        }
    }
}
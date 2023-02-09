//! This shows how to use the TIMG peripheral interrupts.
//! There is TIMG0 which contains a general purpose timer and a watchdog timer.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32c2_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals, TIMG0},
    prelude::*,
    riscv,
    timer::{Timer, Timer0, TimerGroup},
    Rtc,
};
use esp_backtrace as _;

static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C2, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDT.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    interrupt::enable(
        peripherals::Interrupt::TG0_T0_LEVEL,
        interrupt::Priority::Priority1,
    )
    .unwrap();
    timer0.start(500u64.millis());
    timer0.listen();

    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    unsafe {
        riscv::interrupt::enable();
    }

    loop {}
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        esp_println::println!("Interrupt 1");

        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();
        timer0.start(500u64.millis());
    });
}

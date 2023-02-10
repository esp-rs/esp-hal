//! This shows how to use the TIMG peripheral interrupts.
//! There is TIMG0 and TIMG1 each of them containing two general purpose timers
//! and a watchdog timer.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32s2_hal::{
    clock::ClockControl,
    interrupt,
    interrupt::Priority,
    peripherals::{self, Peripherals, TIMG0, TIMG1},
    prelude::*,
    timer::{Timer, Timer0, Timer1, TimerGroup},
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

static TIMER00: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static TIMER01: Mutex<RefCell<Option<Timer<Timer1<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static TIMER10: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));
static TIMER11: Mutex<RefCell<Option<Timer<Timer1<TIMG1>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the TIMG watchdog timer.
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer00 = timer_group0.timer0;
    let mut timer01 = timer_group0.timer1;
    let mut wdt0 = timer_group0.wdt;

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer10 = timer_group1.timer0;
    let mut timer11 = timer_group1.timer1;
    let mut wdt1 = timer_group1.wdt;

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt0.disable();
    wdt1.disable();
    rtc.rwdt.disable();

    interrupt::enable(peripherals::Interrupt::TG0_T0_LEVEL, Priority::Priority2).unwrap();
    interrupt::enable(peripherals::Interrupt::TG0_T1_LEVEL, Priority::Priority2).unwrap();
    interrupt::enable(peripherals::Interrupt::TG1_T0_LEVEL, Priority::Priority3).unwrap();
    interrupt::enable(peripherals::Interrupt::TG1_T1_LEVEL, Priority::Priority3).unwrap();
    timer00.start(500u64.millis());
    timer00.listen();
    timer01.start(2500u64.millis());
    timer01.listen();
    timer10.start(1u64.secs());
    timer10.listen();
    timer11.start(3u64.secs());
    timer11.listen();

    critical_section::with(|cs| {
        TIMER00.borrow_ref_mut(cs).replace(timer00);
        TIMER01.borrow_ref_mut(cs).replace(timer01);
        TIMER10.borrow_ref_mut(cs).replace(timer10);
        TIMER11.borrow_ref_mut(cs).replace(timer11);
    });

    loop {}
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER00.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.start(500u64.millis());
            println!("Interrupt Level 2 - Timer0");
        }
    });
}

#[interrupt]
fn TG0_T1_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER01.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.start(500u64.millis());
            println!("Interrupt Level 2 - Timer1");
        }
    });
}

#[interrupt]
fn TG1_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER10.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.start(500u64.millis());
            println!("Interrupt Level 3 - Timer0");
        }
    });
}

#[interrupt]
fn TG1_T1_LEVEL() {
    critical_section::with(|cs| {
        let mut timer = TIMER11.borrow_ref_mut(cs);
        let timer = timer.as_mut().unwrap();

        if timer.is_interrupt_set() {
            timer.clear_interrupt();
            timer.start(500u64.millis());
            println!("Interrupt Level 3 - Timer1");
        }
    });
}

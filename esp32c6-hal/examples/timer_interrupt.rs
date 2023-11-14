//! This shows how to use the TIMG peripheral interrupts.
//! There is TIMG0 and TIMG1 each of them containing a general purpose timer and
//! a watchdog timer.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32c6_hal::{
    clock::ClockControl,
    interrupt,
    peripherals::{self, Peripherals, TIMG0, TIMG1},
    prelude::*,
    timer::{Timer, Timer0, TimerGroup},
};
use esp_backtrace as _;

static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));
static TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer1 = timer_group1.timer0;

    interrupt::enable(
        peripherals::Interrupt::TG0_T0_LEVEL,
        interrupt::Priority::Priority2,
    )
    .unwrap();

    timer0.start(500u64.millis());

    timer0.listen();

    interrupt::enable(
        peripherals::Interrupt::TG1_T0_LEVEL,
        interrupt::Priority::Priority2,
    )
    .unwrap();

    timer1.start(1u64.secs());
    timer1.listen();

    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
        TIMER1.borrow_ref_mut(cs).replace(timer1);
    });

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

#[interrupt]
fn TG1_T0_LEVEL() {
    critical_section::with(|cs| {
        esp_println::println!("Interrupt 11");

        let mut timer1 = TIMER1.borrow_ref_mut(cs);
        let timer1 = timer1.as_mut().unwrap();

        timer1.clear_interrupt();
        timer1.start(1u64.secs());
    });
}

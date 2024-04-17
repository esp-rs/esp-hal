//! This shows how to use the SYSTIMER peripheral including interrupts.
//!
//! It's an additional timer besides the TIMG peripherals.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    systimer::{Alarm, Periodic, SystemTimer, Target},
    Blocking,
};
use esp_println::println;
use fugit::ExtU32;

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, Blocking, 0>>>> =
    Mutex::new(RefCell::new(None));
static ALARM1: Mutex<RefCell<Option<Alarm<Target, Blocking, 1>>>> = Mutex::new(RefCell::new(None));
static ALARM2: Mutex<RefCell<Option<Alarm<Target, Blocking, 2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    println!("SYSTIMER Current value = {}", SystemTimer::now());

    critical_section::with(|cs| {
        let mut alarm0 = systimer.alarm0.into_periodic();
        alarm0.set_interrupt_handler(systimer_target0);
        alarm0.set_period(1u32.secs());
        alarm0.enable_interrupt(true);

        let mut alarm1 = systimer.alarm1;
        alarm1.set_interrupt_handler(systimer_target1);
        alarm1.set_target(SystemTimer::now() + (SystemTimer::TICKS_PER_SECOND * 2));
        alarm1.enable_interrupt(true);

        let mut alarm2 = systimer.alarm2;
        alarm2.set_interrupt_handler(systimer_target2);
        alarm2.set_target(SystemTimer::now() + (SystemTimer::TICKS_PER_SECOND * 3));
        alarm2.enable_interrupt(true);

        ALARM0.borrow_ref_mut(cs).replace(alarm0);
        ALARM1.borrow_ref_mut(cs).replace(alarm1);
        ALARM2.borrow_ref_mut(cs).replace(alarm2);
    });

    interrupt::enable(Interrupt::SYSTIMER_TARGET0, Priority::Priority1).unwrap();
    interrupt::enable(Interrupt::SYSTIMER_TARGET1, Priority::Priority3).unwrap();
    interrupt::enable(Interrupt::SYSTIMER_TARGET2, Priority::Priority3).unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let delay = Delay::new(&clocks);

    loop {
        delay.delay_millis(500);
    }
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn systimer_target0() {
    println!("Interrupt lvl1 (alarm0)");
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[handler(priority = esp_hal::interrupt::Priority::Priority1)]
fn systimer_target1() {
    println!("Interrupt lvl2 (alarm1)");
    critical_section::with(|cs| {
        ALARM1
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[handler(priority = esp_hal::interrupt::Priority::max())]
fn systimer_target2() {
    println!("Interrupt lvl2 (alarm2)");
    critical_section::with(|cs| {
        ALARM2
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

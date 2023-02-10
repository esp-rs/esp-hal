//! This shows how to use the SYSTIMER peripheral including interrupts.
//! It's an additional timer besides the TIMG peripherals.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32s2_hal::{
    clock::ClockControl,
    interrupt,
    interrupt::Priority,
    peripherals::{self, Peripherals},
    prelude::*,
    systimer::{Alarm, Periodic, SystemTimer, Target},
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, 0>>>> = Mutex::new(RefCell::new(None));
static ALARM1: Mutex<RefCell<Option<Alarm<Target, 1>>>> = Mutex::new(RefCell::new(None));
static ALARM2: Mutex<RefCell<Option<Alarm<Target, 2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let syst = SystemTimer::new(peripherals.SYSTIMER);

    println!("SYSTIMER Current value = {}", SystemTimer::now());

    let alarm0 = syst.alarm0.into_periodic();
    alarm0.set_period(1u32.Hz());
    alarm0.interrupt_enable(true);

    let alarm1 = syst.alarm1;
    alarm1.set_target(SystemTimer::now() + (SystemTimer::TICKS_PER_SECOND * 2));
    alarm1.interrupt_enable(true);

    let alarm2 = syst.alarm2;
    alarm2.set_target(SystemTimer::now() + (SystemTimer::TICKS_PER_SECOND * 3));
    alarm2.interrupt_enable(true);

    critical_section::with(|cs| {
        ALARM0.borrow_ref_mut(cs).replace(alarm0);
        ALARM1.borrow_ref_mut(cs).replace(alarm1);
        ALARM2.borrow_ref_mut(cs).replace(alarm2);
    });

    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET0,
        Priority::Priority1,
    )
    .unwrap();
    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET1,
        Priority::Priority3,
    )
    .unwrap();
    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET2,
        Priority::Priority3,
    )
    .unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    loop {
        delay.delay_ms(500u32);
    }
}

#[interrupt]
fn SYSTIMER_TARGET0() {
    println!("Interrupt lvl1 (alarm0)");
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[interrupt]
fn SYSTIMER_TARGET1() {
    println!("Interrupt lvl3 (alarm1)");
    critical_section::with(|cs| {
        ALARM1
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

#[interrupt]
fn SYSTIMER_TARGET2() {
    println!("Interrupt lvl3 (alarm2)");
    critical_section::with(|cs| {
        ALARM2
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

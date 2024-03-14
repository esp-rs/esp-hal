//! This shows how to use the general purpose timers ETM tasks and events
//! Notice you need to import the traits esp_hal::timer::etm::{TimerEtmEvents, TimerEtmTasks}
//!
//% CHIPS: esp32h2 esp32c6

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::{CriticalSection, Mutex};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    etm::Etm,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, TIMG0},
    prelude::*,
    timer::{
        etm::{TimerEtmEvents, TimerEtmTasks},
        Timer,
        Timer0,
        TimerGroup,
    },
};

static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timg0.timer0;

    // Configure ETM to stop timer0 when alarm is triggered
    let event = timer0.on_alarm();
    let task = timer0.cnt_stop();

    let etm = Etm::new(peripherals.SOC_ETM);

    let channel0 = etm.channel0;

    channel0.setup(&event, &task);

    timer0.set_counter_decrementing(false);
    timer0.reset_counter();
    timer0.set_counter_active(true);

    // Setup alarm at 100ms
    // 80 / 2 (default divider) timer clock cycles == 1 us
    timer0.load_alarm_value(100 * 1_000 * 40);

    // Enable interrupt that will be triggered by the alarm
    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();

    timer0.set_alarm_active(true);

    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    loop {}
}

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();

        // This should display close to 4_000_000
        esp_println::println!("Counter: {}", timer0.now());
    });
}

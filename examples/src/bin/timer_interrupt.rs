//! This shows how to use the TIMG peripheral interrupts.
//!
//! There is TIMG0 which contains a general purpose timer and a watchdog timer.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, TIMG0},
    prelude::*,
    system::SystemControl,
    timer::timg::{Timer, Timer0, TimerGroup},
};

static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0 = timg0.timer0;
    timer0.set_interrupt_handler(tg0_t0_level);

    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();
    timer0.load_value(500u64.millis()).unwrap();
    timer0.start();
    timer0.listen();

    critical_section::with(|cs| {
        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });

    loop {}
}

#[handler]
fn tg0_t0_level() {
    critical_section::with(|cs| {
        esp_println::println!(
            "Interrupt at {} ms",
            esp_hal::time::current_time()
                .duration_since_epoch()
                .to_millis()
        );

        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();
        timer0.load_value(500u64.millis()).unwrap();
        timer0.start();
    });
}

//! This shows how to use the `PeriodicTimer` driver with the TIMG peripheral.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    prelude::*,
    timer::{timg::TimerGroup, AnyTimer, PeriodicTimer},
};

static TIMER0: Mutex<RefCell<Option<PeriodicTimer<AnyTimer>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut timer = PeriodicTimer::new(timg0.timer0);
    timer.set_interrupt_handler(tg0_t0_level);
    timer.start(500.millis()).unwrap();

    critical_section::with(|cs| {
        timer.enable_interrupt(true);
        TIMER0.borrow_ref_mut(cs).replace(timer);
    });

    loop {}
}

#[handler]
fn tg0_t0_level() {
    critical_section::with(|cs| {
        esp_println::println!(
            "Interrupt at {} ms",
            esp_hal::time::now().duration_since_epoch().to_millis()
        );

        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();
    });
}

//! This shows how to spawn a task on the second core.
//!
//! The first core will print the value of a counter which is incremented by the
//! second core.

//% CHIPS: esp32 esp32s3
//% FEATURES: embedded-hal-02

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal_02::timer::CountDown;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
};
use esp_println::println;
use nb::block;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timg0.timer0;

    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer1 = timg1.timer0;

    timer0.start(1u64.secs());
    timer1.start(500u64.millis());

    let counter = Mutex::new(RefCell::new(0u32));

    let mut cpu_control = CpuControl::new(system.cpu_control);
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, || {
            println!("Hello World - Core 1!");
            loop {
                block!(timer1.wait()).unwrap();
                critical_section::with(|cs| {
                    let mut val = counter.borrow_ref_mut(cs);
                    *val = val.wrapping_add(1);
                });
            }
        })
        .unwrap();

    loop {
        block!(timer0.wait()).unwrap();

        let count = critical_section::with(|cs| *counter.borrow_ref(cs));
        println!("Hello World - Core 0! Counter is {}", count);
    }
}

//! This shows how to spawn a task on the second core.
//!
//! The first core will print the value of a counter which is incremented by the
//! second core.

//% CHIPS: esp32 esp32s3

#![no_std]
#![no_main]

use core::{cell::RefCell, ptr::addr_of_mut};

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    cpu_control::{CpuControl, Stack},
    delay::Delay,
    prelude::*,
};
use esp_println::println;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[entry]
fn main() -> ! {
    let (peripherals, clocks) = esp_hal::init(Config::default());

    let delay = Delay::new(&clocks);

    let counter = Mutex::new(RefCell::new(0u32));

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
            println!("Hello World - Core 1!");
            loop {
                delay.delay(500.millis());
                critical_section::with(|cs| {
                    let mut val = counter.borrow_ref_mut(cs);
                    *val = val.wrapping_add(1);
                });
            }
        })
        .unwrap();

    loop {
        delay.delay(1.secs());

        let count = critical_section::with(|cs| *counter.borrow_ref(cs));
        println!("Hello World - Core 0! Counter is {}", count);
    }
}

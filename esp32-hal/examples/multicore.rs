//! This shows how to spawn a task on the second core.
//! The first core will print the value of a counter which is incremented by the
//! second core.

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp32_hal::{
    clock::ClockControl,
    cpu_control::{CpuControl, Stack},
    peripherals::{Peripherals, TIMG1},
    prelude::*,
    timer::{Timer, Timer0, TimerGroup},
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer1 = timer_group1.timer0;

    timer0.start(1u64.secs());
    timer1.start(500u64.millis());

    let counter = Mutex::new(RefCell::new(0));

    let mut cpu_control = CpuControl::new(system.cpu_control);
    let cpu1_fnctn = || {
        cpu1_task(&mut timer1, &counter);
    };
    let _guard = cpu_control
        .start_app_core(unsafe { &mut APP_CORE_STACK }, cpu1_fnctn)
        .unwrap();

    loop {
        block!(timer0.wait()).unwrap();

        let count = critical_section::with(|cs| *counter.borrow_ref(cs));
        println!("Hello World - Core 0! Counter is {}", count);
    }
}

fn cpu1_task(
    timer: &mut Timer<Timer0<TIMG1>>,
    counter: &critical_section::Mutex<RefCell<i32>>,
) -> ! {
    println!("Hello World - Core 1!");
    loop {
        block!(timer.wait()).unwrap();

        critical_section::with(|cs| {
            let new_val = counter.borrow_ref_mut(cs).wrapping_add(1);
            *counter.borrow_ref_mut(cs) = new_val;
        });
    }
}

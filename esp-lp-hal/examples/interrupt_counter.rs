//! ULP interrupt-based counter example.
//! Increments a 32 bit counter value at a known point in memory,
//! whenever the ULP program is run. If GPIO0 is pressed, reset the counter.

//% CHIPS: esp32s3

#![no_std]
#![no_main]
extern crate panic_halt;

use esp_lp_hal::{
    gpio::{self, Event, Input, Io},
    interrupt,
    pac::Peripherals,
    prelude::*,
};

// Shared memory address.
const ADDRESS: u32 = 0x1000;

fn increment_counter() {
    let ptr = ADDRESS as *mut u32;
    let i = unsafe { ptr.read_volatile() };
    unsafe {
        ptr.write_volatile(i + 1);
    }
}
fn reset_counter() {
    let ptr = ADDRESS as *mut u32;
    unsafe {
        ptr.write_volatile(0);
    }
}

#[entry]
fn main(mut boot_button: Input<0>) {
    // Get the io peripheral, and bind a handler to it.
    // let peripherals = Peripherals::take().unwrap(); // Requires critical_section
    let peripherals = unsafe { Peripherals::steal() };
    let mut io = Io::new(peripherals.RTC_IO);
    io.set_interrupt_handler(gpio_interrupt_handler);
    boot_button.listen(Event::FallingEdge);

    increment_counter();
}

#[handler]
fn gpio_interrupt_handler() {
    // TODO: Create an enum for each GPIO pin? Maybe?
    let status = gpio::gpio_interrupt_status();
    if status & (0b1) != 0 {
        reset_counter();
    }
    gpio::gpio_interrupt_clear(status);
}

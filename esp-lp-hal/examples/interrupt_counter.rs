//! ULP interrupt-based counter example.
//! Increments a 32 bit counter value at a known point in memory, whenever the ULP program is run.
//! If GPIO0 is pressed, resets the counter.

#![no_std]
#![no_main]

extern crate panic_halt;

use core::cell::RefCell;

use critical_section::Mutex;
use esp_lp_hal::{
    gpio::{Event, Input, Io},
    interrupt::{self, Interrupt},
    pac::Peripherals,
    prelude::*,
};

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

// NOTE: Normally this would contain Option<Input<0>>,
//       but for some reason the ULP core is crashing when
//       .unwrap() is used on anything...
static BUTTON: Mutex<RefCell<Input<0>>> = Mutex::new(RefCell::new(Input::<0>::new()));

#[entry]
fn main(mut button: Input<0>) {
    let peripherals = Peripherals::take();
    let mut io = Io::new(peripherals.RTC_IO);
    io.set_interrupt_handler(gpio_interrupt_handler);

    interrupt::bind_handler(Interrupt::RISCV_START_INT, startup_interrupt_handler);

    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        *BUTTON.borrow_ref_mut(cs) = button;
    });
}

#[handler]
fn gpio_interrupt_handler() {
    if critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).is_interrupt_set()) {
        // The button was the source of the interrupt
        reset_counter();
    }

    // Clear the interrupt
    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).clear_interrupt());
}

#[handler]
fn startup_interrupt_handler() {
    increment_counter();
    interrupt::disable(Interrupt::RISCV_START_INT);
}

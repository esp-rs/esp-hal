//! ULP GPIO wakeup example.
//! Increments a 32 bit counter value at a known point in memory, whenever the ULP program is woken
//! up. The HP core will configure GPIO0 to wake up the ULP core, when pressed.

//% CHIPS: esp32s3 esp32s2

#![no_std]
#![no_main]

extern crate panic_halt;

use esp_lp_hal::{
    gpio::{Input, gpio_wakeup_clear, gpio_wakeup_enable},
    prelude::*,
};

const ADDRESS: usize = 0x1000;

#[entry]
fn main(mut _button: Input<0>) {
    // Clear the global GPIO wake-up flag
    gpio_wakeup_clear();

    // Increment the counter
    unsafe {
        let counter = ADDRESS as *mut u32;
        counter.write_volatile(counter.read_volatile() + 1);
    }

    // Re-enable the global GPIO wakeup flag
    gpio_wakeup_enable(true);
}

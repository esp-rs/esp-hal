//! ULP GPIO-wakeup example.
//! Increments a 32 bit counter value, while GPIO0 is pressed.

//% CHIPS: esp32s3

#![no_std]
#![no_main]

extern crate panic_halt;

use esp_lp_hal::{gpio::Input, prelude::*};

const ADDRESS: usize = 0x1000;

#[entry]
fn main(mut _button: Input<0>) {
    // Clear the GPIO wake-up flag
    esp_lp_hal::gpio::gpio_wakeup_clear();

    // Increment counter
    let ptr = ADDRESS as *mut u32;
    unsafe {
        let count = ptr.read_volatile();
        ptr.write_volatile(count + 1);
    }

    // Re-enable the wakeup bit
    esp_lp_hal::gpio::gpio_wakeup_enable(true);
}

//! ULP interrupt-based counter example.
//! Increments a 32 bit counter value at a known point in memory,
//! whenever the ULP program is run. If GPIO0 is pressed, reset the counter.

//% CHIPS: esp32s3

#![no_std]
#![no_main]
extern crate panic_halt;
// TODO: I'd prefer if these were proc-macros, which could be used as function attributes.
//       This would 1) look nicer, and 2) provide better discoverability/type-hinting of the
// interrupts available for use.
use esp_lp_hal::{
    gpio::{Event, Input},
    interrupts::{GpioInterruptPin, SensInterruptStatus, gpio_interrupt, sens_interrupt},
    prelude::*,
};

// Shared memory address.
const ADDRESS: u32 = 0x1000;

pub fn on_start(status: SensInterruptStatus) {
    if status == SensInterruptStatus::RISCV_START_INT {
        // Did we get a startup interrupt? If so, increment counter
        let ptr = ADDRESS as *mut u32;
        let i = unsafe { ptr.read_volatile() };
        unsafe {
            ptr.write_volatile(i + 1);
        }
    }
}

sens_interrupt!(on_start);

pub fn on_button(pin: GpioInterruptPin) {
    // Reset the counter on GPIO0
    if pin == 0 {
        let ptr = ADDRESS as *mut u32;
        unsafe {
            ptr.write_volatile(0);
        }
    }
}

gpio_interrupt!(on_button);

#[entry]
fn main(mut boot_pin: Input<0>) {
    // Enable start-up interrupt, called shortly after boot.
    unsafe { &*esp_lp_hal::pac::SENS::PTR }
        .sar_cocpu_int_ena()
        .write(|w| w.sar_cocpu_start_int_ena().set_bit());

    boot_pin.listen(Event::FallingEdge);
}

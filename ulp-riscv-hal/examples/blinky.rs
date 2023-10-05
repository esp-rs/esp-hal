//! Counts a 32 bit value at 0x5000_0400 (0x400 from the ULP's point of view)
//! and blinks GPIO 1.
//!
//! Make sure the RTC RAM is cleared before loading the code.

#![no_std]
#![no_main]

use panic_halt as _;
use ulp_riscv_hal::{
    delay::Delay,
    gpio::{GpioPin, Output, PushPull},
    prelude::*,
};

#[entry]
fn main(mut gpio1: GpioPin<Output<PushPull>, 1>) -> ! {
    let mut i: u32 = 0;

    let ptr = 0x400 as *mut u32;
    let mut delay = Delay::new();

    loop {
        i = i.wrapping_add(1u32);
        unsafe {
            ptr.write_volatile(i);
        }

        gpio1.set_high().unwrap();
        delay.delay_ms(500);

        gpio1.set_low().unwrap();
        delay.delay_ms(500);
    }
}

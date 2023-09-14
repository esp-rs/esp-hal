//! Counts a 32 bit value at 0x5000_2000 and blinks GPIO 1.
//! Make sure the LP RAM is cleared before loading the code.

#![no_std]
#![no_main]

use esp32c6_lp_hal::{
    delay::Delay,
    gpio::{GpioPin, Output, PushPull},
    prelude::*,
};
use panic_halt as _;

#[entry]
fn main(mut gpio1: GpioPin<Output<PushPull>, 1>) -> ! {
    let mut i: u32 = 0;

    let ptr = 0x5000_2000 as *mut u32;
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

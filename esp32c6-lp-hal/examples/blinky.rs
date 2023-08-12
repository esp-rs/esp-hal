//! Counts a 32 bit value at 0x5000_2000 and blinks GPIO 1.
//! Make sure the LP RAM is cleared before loading the code.

#![no_std]
#![no_main]

use esp32c6_lp_hal::{gpio::Io, prelude::*};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut i: u32 = 0;

    let peripherals = esp32c6_lp::Peripherals::take().unwrap();

    let io = Io::new(peripherals.LP_IO);
    let mut gpio1 = io.gpio1.into_output();

    let ptr = 0x5000_2000 as *mut u32;
    let mut delay = delay::Delay::new();

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

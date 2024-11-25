//! Counts a 32 bit value at a known point in memory, and blink GPIO1.
//!
//! When using the ESP32-C6's LP core, this address in memory is `0x5000_2000`.
//!
//! When using the ESP32-S2 or ESP32-S3's ULP core, this address in memory is
//! `0x5000_0400` (but is `0x400`` from the ULP's point of view!).
//!
//! Make sure the LP RAM is cleared before loading the code.

#![no_std]
#![no_main]

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use esp_lp_hal::{delay::Delay, gpio::Output, prelude::*};
use panic_halt as _;

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c6")] {
        const ADDRESS: u32 = 0x5000_2000;
    } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
        const ADDRESS: u32 = 0x400;
    }
}

#[entry]
fn main(mut gpio1: Output<1>) -> ! {
    let mut i: u32 = 0;

    let ptr = ADDRESS as *mut u32;

    loop {
        i = i.wrapping_add(1u32);
        unsafe {
            ptr.write_volatile(i);
        }

        gpio1.set_high().unwrap();
        Delay.delay_ms(500);

        gpio1.set_low().unwrap();
        Delay.delay_ms(500);
    }
}

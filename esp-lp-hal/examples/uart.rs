//! Uses `LP_UART` and logs "Hello World from LP Core".
//!
//! Uses GPIO4 for RX and GPIO5 for TX. GPIOs can't be changed.
//!
//! It is neccessary to use Serial-Uart bridge connected to TX and RX to see
//! logs from LP_UART. Make sure the LP RAM is cleared before loading the code.

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use core::fmt::Write;

use embedded_hal::delay::DelayNs;
use esp_lp_hal::{delay::Delay, prelude::*, uart::LpUart};
use panic_halt as _;

#[entry]
fn main(mut uart: LpUart) -> ! {
    loop {
        writeln!(uart, "Hello World from LP Core").unwrap();
        Delay.delay_ms(1000);
    }
}

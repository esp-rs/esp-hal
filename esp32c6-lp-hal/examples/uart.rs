#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c6_lp_hal::{
    delay::Delay,
    prelude::*,
    uart::{LpUart},
};
use panic_halt as _;

#[entry]
fn main(mut uart: LpUart) -> ! {
    let _peripherals = esp32c6_lp::Peripherals::take().unwrap();

    let mut delay = Delay::new();
    loop {
        writeln!(uart, "Hello World from LP Core").unwrap();
        delay.delay_ms(1500);
    }
}

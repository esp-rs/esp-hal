#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s3_hal::{pac::Peripherals, prelude::*, Serial, Timer};
use nb::block;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    // Disable watchdog timer
    timer0.disable();

    timer0.start(40_000_000u64);

    loop {
        writeln!(serial0, "Hello world!").unwrap();
        block!(timer0.wait()).unwrap();
    }
}

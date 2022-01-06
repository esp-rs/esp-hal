#![no_std]
#![no_main]

use esp32_hal::{gpio::IO, pac, prelude::*, Timer};
use nb::block;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut timer0 = Timer::new(peripherals.TIMG0);

    timer0.disable();

    let mut led = io.pins.gpio15.into_push_pull_output();
    led.set_high().unwrap();

    timer0.start(10_000_000u64);

    loop {
        led.toggle().unwrap();
        block!(timer0.wait()).unwrap();
    }
}

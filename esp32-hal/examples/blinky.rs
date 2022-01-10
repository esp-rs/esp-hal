#![no_std]
#![no_main]

use esp32_hal::{gpio::IO, pac::Peripherals, prelude::*, Timer};
use nb::block;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio15.into_push_pull_output();
    let mut timer0 = Timer::new(peripherals.TIMG0);

    // Disable watchdog timer
    timer0.disable();

    led.set_high().unwrap();
    timer0.start(10_000_000u64);

    loop {
        led.toggle().unwrap();
        block!(timer0.wait()).unwrap();
    }
}

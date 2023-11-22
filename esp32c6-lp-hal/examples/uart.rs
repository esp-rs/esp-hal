#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c6_lp_hal::{
    delay::Delay,
    gpio::{Floating, GpioPin, Input, Output, PushPull},
    prelude::*,
    uart::Uart,
};
use panic_halt as _;

#[entry]
fn main(_tx: GpioPin<Output<PushPull>, 5>, _rx: GpioPin<Input<Floating>, 4>) -> ! {
    let peripherals = esp32c6_lp::Peripherals::take().unwrap();

    let mut delay = Delay::new();
    let mut uart = Uart::new(peripherals.LP_UART);

    loop {
        writeln!(uart, "Hello, world!").unwrap();
        delay.delay_ms(500);
    }
}

//! Iterates over an array of LEDs, demonstrating how to use AnyPin and FlexPins to drive GPIOs.
//!
//! The following wiring is assumed:
//! - LED => GPIO0
//! - LED => GPIO1
//! - LED => GPIO2

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    gpio::any_pin::AnyPin,
    gpio::Flex,
};
use core::primitive::u32;

pub struct FlexIo<'a> {
    pub gpio0: Flex<'a, AnyPin<'a>>,
    pub gpio1: Flex<'a, AnyPin<'a>>,
    pub gpio2: Flex<'a, AnyPin<'a>>,
}

impl<'a> FlexIo<'a> {
    pub fn new(io: Io) -> Self {
        FlexIo {
            gpio0: Flex::new(AnyPin::new(io.pins.gpio0)),
            gpio1: Flex::new(AnyPin::new(io.pins.gpio1)),
            gpio2: Flex::new(AnyPin::new(io.pins.gpio2)),
        }
    }
    pub fn get_pin(&mut self, index: u32) -> &mut Flex<'a, AnyPin<'a>> {
        match index {
            0 => &mut self.gpio0,
            1 => &mut self.gpio1,
            2 => &mut self.gpio2,
            _ => panic!("No such pin"),
        }
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let delay = Delay::new(&clocks);
    
    let mut flexIo = FlexIo::new(io);
    // You can also use set_as_input() to use those GPIOs easily as input too 
    flexIo.gpio0.set_as_output();
    flexIo.gpio1.set_as_output();
    flexIo.gpio2.set_as_output();

    loop {
        for n in 0..3 {
            flexIo.get_pin(n).toggle();
            delay.delay_millis(250);
        }
    }
}
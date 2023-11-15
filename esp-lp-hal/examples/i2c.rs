#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c6_lp_hal::{
    delay::Delay,
    gpio::{GpioPin, InOut, Unknown},
    i2c::*,
    prelude::*,
};
use fugit::*;
use liquid_crystal::{prelude::*, I2C as I2C_interface};
use panic_halt as _;

#[entry]
fn main(_sda: GpioPin<InOut, 6>, _scl: GpioPin<InOut, 7>) -> ! {
    let data = (0x5000_2000) as *mut u32;

    unsafe {
        let peripherals = esp32c6_lp::Peripherals::take().unwrap();

        let mut delay = Delay::new();

        data.write_volatile(0x0000_0001);

        // Initializing the I2C interface
        let mut i2c = esp32c6_lp_hal::i2c::I2C::new(peripherals.LP_I2C, 100u32.kHz());
        data.write_volatile(0x0000_0002);

        // let mut interface = I2C_interface::new(i2c, 0x01);

        // let mut lcd = LiquidCrystal::new(&mut interface, Bus4Bits, LCD16X2);
        // data.write_volatile(0x0000_0004);
        // lcd.begin(&mut delay);
        // data.write_volatile(0x0000_0005);
        // lcd.write(&mut delay, Text("LP Core works!"));

        loop {
            i2c.write(0x33, "abcde".as_bytes());
        }
    }
}

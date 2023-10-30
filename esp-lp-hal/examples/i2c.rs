#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c6_lp_hal::{delay::Delay, i2c::*, prelude::*};
use fugit::*;
use liquid_crystal::{prelude::*, I2C as I2C_interface};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let peripherals = esp32c6_lp::Peripherals::take().unwrap();
    let mut delay = Delay::new();

    // Initializing the I2C interface
    let mut i2c = I2C::new(peripherals.LP_I2C, 100u32.kHz());
    let mut interface = I2C_interface::new(i2c, 0x27);
    let mut lcd = LiquidCrystal::new(&mut interface, Bus4Bits, LCD16X2);

    lcd.begin(&mut delay);
    lcd.write(&mut delay, Text("LP CORE WORKS!"));

    loop {}
}

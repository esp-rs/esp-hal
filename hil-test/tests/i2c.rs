//! I2C test
//!
//! Folowing pins are used:
//! SCL    GPIO4
//! SDA    GPIO7

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    system::SystemControl,
    Blocking,
};
use hil_test as _;

struct Context {
    i2c: I2C<'static, I2C0, Blocking>,
}
#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_ne;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (sda, scl) = hil_test::i2c_pins!(io);

        // Create a new peripheral object with the described wiring and standard
        // I2C clock speed:
        let i2c = I2C::new(peripherals.I2C0, sda, scl, 100.kHz(), &clocks);

        Context { i2c }
    }

    #[test]
    #[timeout(3)]
    fn test_read_cali(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        ctx.i2c.write_read(0x77, &[0xaa], &mut read_data).ok();

        assert_ne!(read_data, [0u8; 22])
    }
}

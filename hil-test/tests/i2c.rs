//! I2C test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    i2c::master::{Config, Error, I2c, Operation},
    Async,
    Blocking,
};
use hil_test as _;

struct Context {
    i2c: I2c<'static, Blocking>,
}

fn _async_driver_is_compatible_with_blocking_ehal() {
    fn _with_driver(driver: I2c<'static, Async>) {
        _with_ehal(driver);
    }

    fn _with_ehal(_: impl embedded_hal::i2c::I2c) {}
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (sda, scl) = hil_test::i2c_pins!(peripherals);

        // Create a new peripheral object with the described wiring and standard
        // I2C clock speed:
        let i2c = I2c::new(peripherals.I2C0, Config::default())
            .with_sda(sda)
            .with_scl(scl);

        Context { i2c }
    }

    #[test]
    #[timeout(3)]
    fn empty_read_write_returns_ack_error(mut ctx: Context) {
        assert_eq!(ctx.i2c.write(0xab, &[]), Err(Error::AckCheckFailed));
    }

    #[test]
    #[timeout(3)]
    fn test_read_cali(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        ctx.i2c.write_read(0x55, &[0xaa], &mut read_data).ok();

        // do the real read which should succeed
        ctx.i2c.write_read(0x77, &[0xaa], &mut read_data).ok();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    #[timeout(3)]
    fn test_read_cali_with_transactions(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        ctx.i2c
            .transaction(
                0x77,
                &mut [Operation::Write(&[0xaa]), Operation::Read(&mut read_data)],
            )
            .ok();

        assert_ne!(read_data, [0u8; 22])
    }
}

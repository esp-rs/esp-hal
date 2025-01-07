//! I2C test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

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

const DUT_ADDRESS: u8 = 0x77;
const NON_EXISTENT_ADDRESS: u8 = 0x6b;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (sda, scl) = hil_test::i2c_pins!(peripherals);

        // Create a new peripheral object with the described wiring and standard
        // I2C clock speed:
        let i2c = I2c::new(peripherals.I2C0, Config::default())
            .unwrap()
            .with_sda(sda)
            .with_scl(scl);

        Context { i2c }
    }

    #[test]
    fn empty_write_returns_ack_error_for_unknown_address(mut ctx: Context) {
        assert_eq!(
            ctx.i2c.write(NON_EXISTENT_ADDRESS, &[]),
            Err(Error::AckCheckFailed)
        );
        assert_eq!(ctx.i2c.write(DUT_ADDRESS, &[]), Ok(()));
    }

    #[test]
    fn test_read_cali(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        ctx.i2c
            .write_read(NON_EXISTENT_ADDRESS, &[0xaa], &mut read_data)
            .ok();

        // do the real read which should succeed
        ctx.i2c
            .write_read(DUT_ADDRESS, &[0xaa], &mut read_data)
            .ok();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    fn test_read_cali_with_transactions(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        ctx.i2c
            .transaction(
                DUT_ADDRESS,
                &mut [Operation::Write(&[0xaa]), Operation::Read(&mut read_data)],
            )
            .ok();

        assert_ne!(read_data, [0u8; 22])
    }
}

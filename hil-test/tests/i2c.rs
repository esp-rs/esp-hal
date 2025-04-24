//! I2C test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    Blocking,
    i2c::master::{AcknowledgeCheckFailedReason, Config, Error, I2c, I2cAddress, Operation},
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
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
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
    fn invalid_address_returns_error(mut ctx: Context) {
        assert_eq!(
            ctx.i2c.write(0x80, &[]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
        assert_eq!(
            ctx.i2c.read(0x80, &mut [0; 1]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
        assert_eq!(
            // We can even pass arrays, not just slices! That isn't meaningful for the read buffer,
            // but it's possible.
            ctx.i2c.write_read(0x80, [0x77], [0; 1]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
    }

    #[test]
    fn empty_write_returns_ack_error_for_unknown_address(mut ctx: Context) {
        // on some chips we can determine the ack-check-failed reason but not on all
        // chips
        cfg_if::cfg_if! {
            if #[cfg(any(esp32,esp32s2,esp32c2,esp32c3))] {
                assert_eq!(
                    ctx.i2c.write(NON_EXISTENT_ADDRESS, &[]),
                    Err(Error::AcknowledgeCheckFailed(
                        AcknowledgeCheckFailedReason::Unknown
                    ))
                );
            } else {
                assert_eq!(
                    ctx.i2c.write(NON_EXISTENT_ADDRESS, &[]),
                    Err(Error::AcknowledgeCheckFailed(
                        AcknowledgeCheckFailedReason::Address
                    ))
                );
            }
        }

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

    #[test]
    async fn async_empty_write_returns_ack_error_for_unknown_address(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        // on some chips we can determine the ack-check-failed reason but not on all
        // chips
        cfg_if::cfg_if! {
            if #[cfg(any(esp32,esp32s2,esp32c2,esp32c3))] {
                assert_eq!(
                    i2c.write_async(NON_EXISTENT_ADDRESS, &[]).await,
                    Err(Error::AcknowledgeCheckFailed(
                        AcknowledgeCheckFailedReason::Unknown
                    ))
                );
            } else {
                assert_eq!(
                    i2c.write_async(NON_EXISTENT_ADDRESS, &[]).await,
                    Err(Error::AcknowledgeCheckFailed(
                        AcknowledgeCheckFailedReason::Address
                    ))
                );
            }
        }

        assert_eq!(i2c.write_async(DUT_ADDRESS, &[]).await, Ok(()));
    }

    #[test]
    async fn async_test_read_cali(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 22];

        // have a failing read which might could leave the peripheral in an undesirable
        // state
        i2c.write_read_async(NON_EXISTENT_ADDRESS, &[0xaa], &mut read_data)
            .await
            .ok();

        // do the real read which should succeed
        i2c.write_read_async(DUT_ADDRESS, &[0xaa], &mut read_data)
            .await
            .ok();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    async fn async_test_read_cali_with_transactions(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [Operation::Write(&[0xaa]), Operation::Read(&mut read_data)],
        )
        .await
        .ok();

        assert_ne!(read_data, [0u8; 22])
    }

    // This is still an issue on ESP32-S2
    #[cfg(not(esp32s2))]
    #[test]
    async fn async_test_timeout_when_scl_kept_low(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        esp_hal::gpio::InputSignal::I2CEXT0_SCL.connect_to(&esp_hal::gpio::Level::Low);

        let mut read_data = [0u8; 22];
        // will run into an error but it should return at least
        i2c.write_read_async(DUT_ADDRESS, &[0xaa], &mut read_data)
            .await
            .ok();
    }
}

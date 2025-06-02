//! I2C test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use embassy_futures::select::select;
use embassy_time::{Duration, Ticker};
use esp_hal::{
    Async,
    Blocking,
    i2c::master::{AcknowledgeCheckFailedReason, Config, Error, I2c, I2cAddress, Operation},
    interrupt::{
        Priority,
        software::{SoftwareInterrupt, SoftwareInterruptControl},
    },
};
use esp_hal_embassy::InterruptExecutor;
use hil_test::mk_static;

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    interrupt: SoftwareInterrupt<'static, 1>,
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
const READ_DATA_COMMAND: &[u8] = &[0xaa];

#[embassy_executor::task]
async fn waiting_blocking_task() {
    esp_hal::delay::Delay::new().delay_millis(10);
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        hil_test::init_embassy!(peripherals, 2);
        let (sda, scl) = hil_test::i2c_pins!(peripherals);

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        // Create a new peripheral object with the described wiring and standard
        // I2C clock speed:
        let i2c = I2c::new(peripherals.I2C0, Config::default())
            .unwrap()
            .with_sda(sda)
            .with_scl(scl);

        Context {
            i2c,
            interrupt: sw_ints.software_interrupt1,
        }
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
            ctx.i2c.write_read(0x80, &[0x77], &mut [0; 1]),
            Err(Error::AddressInvalid(I2cAddress::SevenBit(0x80)))
        );
    }

    #[test]
    fn empty_write_returns_ack_error_for_unknown_address(mut ctx: Context) {
        // on some chips we can determine the ack-check-failed reason but not on all
        // chips
        let reason = if cfg!(any(esp32, esp32s2, esp32c2, esp32c3)) {
            AcknowledgeCheckFailedReason::Unknown
        } else {
            AcknowledgeCheckFailedReason::Address
        };

        assert_eq!(
            ctx.i2c.write(NON_EXISTENT_ADDRESS, &[]),
            Err(Error::AcknowledgeCheckFailed(reason))
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
            .expect_err("Expected error for non-existent address");

        // do the real read which should succeed
        ctx.i2c
            .write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    fn test_read_cali_with_transactions(mut ctx: Context) {
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        ctx.i2c
            .transaction(
                DUT_ADDRESS,
                &mut [
                    Operation::Write(READ_DATA_COMMAND),
                    Operation::Read(&mut read_data),
                ],
            )
            .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    async fn async_empty_write_returns_ack_error_for_unknown_address(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        // on some chips we can determine the ack-check-failed reason but not on all
        // chips
        let reason = if cfg!(any(esp32, esp32s2, esp32c2, esp32c3)) {
            AcknowledgeCheckFailedReason::Unknown
        } else {
            AcknowledgeCheckFailedReason::Address
        };

        let should_fail = i2c.write_async(NON_EXISTENT_ADDRESS, &[]).await;
        assert_eq!(should_fail, Err(Error::AcknowledgeCheckFailed(reason)));

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
            .expect_err("Expected error for non-existent address");

        // do the real read which should succeed
        i2c.write_read_async(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .await
            .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    async fn async_test_read_cali_with_transactions(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 22];

        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [
                Operation::Write(READ_DATA_COMMAND),
                Operation::Read(&mut read_data),
            ],
        )
        .await
        .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    async fn async_cancellation(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();
        let mut read_data = [0u8; 22];

        // Start a transaction that will be cancelled.
        select(
            i2c.transaction_async(
                DUT_ADDRESS,
                &mut [
                    Operation::Write(READ_DATA_COMMAND),
                    Operation::Read(&mut read_data),
                ],
            ),
            async {
                // Let the transaction run for a tiny bit.
                for _ in 0..10 {
                    embassy_futures::yield_now().await;
                }
            },
        )
        .await;

        // do the real read which should succeed
        i2c.transaction_async(
            DUT_ADDRESS,
            &mut [
                Operation::Write(READ_DATA_COMMAND),
                Operation::Read(&mut read_data),
            ],
        )
        .await
        .unwrap();

        assert_ne!(read_data, [0u8; 22])
    }

    #[test]
    async fn test_timeout_when_scl_kept_low(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        esp_hal::gpio::InputSignal::I2CEXT0_SCL.connect_to(&esp_hal::gpio::Level::Low);

        let mut read_data = [0u8; 22];
        // will run into an error but it should return at least
        i2c.write_read(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .expect_err("Expected timeout error");
    }

    #[test]
    async fn async_test_timeout_when_scl_kept_low(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        esp_hal::gpio::InputSignal::I2CEXT0_SCL.connect_to(&esp_hal::gpio::Level::Low);

        let mut read_data = [0u8; 22];
        // will run into an error but it should return at least
        i2c.write_read_async(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data)
            .await
            .expect_err("Expected timeout error");
    }

    #[test]
    #[timeout(10)]
    async fn no_timeout_when_preempted_for_long_time(ctx: Context) {
        let mut i2c = ctx.i2c.into_async();

        let interrupt_executor =
            mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));

        let spawner = interrupt_executor.start(Priority::Priority3);

        let mut ticker = Ticker::every(Duration::from_millis(10));
        for i in 0..100 {
            let mut read_data = [0u8; 22];
            let result = embassy_futures::join::join(
                i2c.write_read_async(DUT_ADDRESS, READ_DATA_COMMAND, &mut read_data),
                async {
                    for _ in 0..4 {
                        spawner.must_spawn(waiting_blocking_task());
                        embassy_futures::yield_now().await;
                    }
                },
            )
            .await;

            assert!(
                result.0.is_ok(),
                "I2C read failed: {:?} in iteration {}",
                result.0,
                i
            );

            ticker.next().await;
        }
    }
}

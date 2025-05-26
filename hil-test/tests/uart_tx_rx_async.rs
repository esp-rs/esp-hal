//! UART TX/RX Async Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    uart::{self, UartRx, UartTx},
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    rx: UartRx<'static, Async>,
    tx: UartTx<'static, Async>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use embassy_futures::{
        join::join,
        select::{Either, select},
    };
    use embassy_time::{Duration, Timer};
    use embedded_io_async::Write;
    use esp_hal::{
        timer::timg::TimerGroup,
        uart::{RxConfig, RxError},
    };

    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_hal_embassy::init(timg0.timer0);

        let tx = UartTx::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .into_async();
        let rx = UartRx::new(peripherals.UART1, uart::Config::default())
            .unwrap()
            .with_rx(rx)
            .into_async();

        Context { rx, tx }
    }

    #[test]
    async fn test_write_read(mut ctx: Context) {
        let byte = [0x42];
        let mut read = [0u8; 1];

        ctx.tx.flush_async().await.unwrap();
        ctx.tx.write_async(&byte).await.unwrap();
        let _ = ctx.rx.read_async(&mut read).await;

        assert_eq!(read, byte);
    }

    #[test]
    async fn write_async_does_not_return_0(mut ctx: Context) {
        let bytes = [0; 200];

        for i in 0..50 {
            let written = ctx.tx.write_async(&bytes).await.unwrap();
            assert_ne!(written, 0, "Iteration {}", i);
        }
    }

    #[test]
    async fn flush_async_does_not_return_prematurely(mut ctx: Context) {
        // Force TX_DONE to be set
        ctx.tx.write_async(&[0u8; 10]).await.unwrap();

        let mut read = [0u8; 10];
        ctx.rx.read_exact_async(&mut read).await.unwrap();

        // The flush should not return until the data is actually sent, regardless of
        // previous TX_DONE status.
        for _ in 0..10 {
            ctx.tx.write_async(&[1u8; 10]).await.unwrap();
            ctx.tx.flush_async().await.unwrap();

            let read_count = ctx.rx.read_buffered(&mut read).unwrap();

            assert_eq!(read_count, 10);
            assert_eq!(&read, &[1u8; 10]);
        }
    }

    #[test]
    async fn flush_async_does_not_return_prematurely_even_for_short_writes(mut ctx: Context) {
        let mut read = [0u8; 2];
        // The flush should not return until the data is actually sent, regardless of
        // previous TX_DONE status.
        for i in 0..10 {
            ctx.tx.write_async(&[i as u8]).await.unwrap();
            ctx.tx.flush_async().await.unwrap();

            let read_count = ctx.rx.read_buffered(&mut read).unwrap();

            assert_eq!(read_count, 1);
            assert_eq!(read[0], i as u8);
        }
    }

    #[test]
    async fn rx_overflow_is_detected(mut ctx: Context) {
        let mut to_send: &[u8] = &[0; 250];
        let mut read = [0u8; 1];

        while !to_send.is_empty() {
            let written = ctx.tx.write_async(to_send).await.unwrap();
            to_send = &to_send[written..];
        }
        ctx.tx.flush_async().await.unwrap();

        // The read is supposed to return an error because the FIFO is just 128 bytes
        // long.
        let res = ctx.rx.read_async(&mut read).await;
        assert!(matches!(res, Err(RxError::FifoOverflowed)), "{:?}", res);
    }

    #[test]
    async fn flushing_after_overflow_remains_consistent(mut ctx: Context) {
        let mut read = [0u8; 1];

        ctx.tx.write_all(&[0; 350]).await.unwrap();
        ctx.tx.flush_async().await.unwrap();

        // The read is supposed to return an error because the FIFO is just 128 bytes
        // long.
        let res = ctx.rx.read_async(&mut read).await;
        assert!(matches!(res, Err(RxError::FifoOverflowed)), "{:?}", res);

        // Now we should be able to write and read back the same data.
        for _ in 0..5 {
            ctx.tx.write_all(&[1, 2, 3, 4]).await.unwrap();

            let mut read = [0u8; 4];
            ctx.rx.read_exact_async(&mut read).await.unwrap();

            assert_eq!(&read, &[1, 2, 3, 4]);
        }

        // Can we still handle a full FIFO?
        // Join two futures to ensure that `read_async` is called first, which should
        // set up waiting for the FIFO full interrupt.
        let mut read = [0u8; 128];
        let read = async {
            // This read should return as many bytes as the FIFO threshold, which is 120
            // bytes by default.
            let read_count = ctx.rx.read_async(&mut read).await.unwrap();
            assert_eq!(read_count, 120);

            ctx.rx.read_exact_async(&mut read[120..]).await.unwrap();
            assert_eq!(&read, &[1; 128]);
        };
        let write = async { ctx.tx.write_all(&[1; 128]).await.unwrap() };

        embassy_futures::join::join(read, write).await;
    }

    #[test]
    async fn read_returns_with_partial_data_if_read_times_out(mut ctx: Context) {
        let mut data = [0u8; 16];

        // We write 4 bytes, but read 16. This should return 4 bytes because the default
        // RX timeout is 10 symbols' worth of time.
        let (read, _) = join(ctx.rx.read_async(&mut data), async {
            ctx.tx.write_async(&[1, 2, 3, 4]).await.unwrap();
            ctx.tx.flush_async().await.unwrap();
        })
        .await;

        assert_eq!(read, Ok(4));
        assert_eq!(&data[..4], &[1, 2, 3, 4]);
    }

    #[test]
    async fn read_exact_does_not_return_partial_data(mut ctx: Context) {
        let mut data = [0u8; 8];

        // Write 4 bytes
        ctx.tx.write_async(&[1, 2, 3, 4]).await.unwrap();
        ctx.tx.flush_async().await.unwrap();

        // read_exact should either read exactly 8 bytes, or time out.
        let should_timeout = select(
            // Wait for 8 bytes or FIFO threshold which is more.
            ctx.rx.read_exact_async(&mut data),
            // We expect to time out
            Timer::after(Duration::from_secs(1)),
        )
        .await;

        assert!(matches!(should_timeout, Either::Second(_)));
    }

    #[test]
    async fn read_does_not_resolve_when_there_is_not_enough_data_and_no_timeout(mut ctx: Context) {
        let mut data = [0u8; 8];

        ctx.rx
            .apply_config(&uart::Config::default().with_rx(RxConfig::default().with_timeout_none()))
            .unwrap();
        // Default RX FIFO threshold is higher than 8 bytes, so we should be able to
        // read all 8 bytes in one go.
        let (should_timeout, _) = join(
            select(
                // Wait for 8 bytes or FIFO threshold which is more.
                ctx.rx.read_async(&mut data),
                // We expect to time out
                Timer::after(Duration::from_secs(1)),
            ),
            async {
                // Write 4 bytes
                ctx.tx.write_async(&[1, 2, 3, 4]).await.unwrap();
                ctx.tx.flush_async().await.unwrap();
            },
        )
        .await;

        assert!(matches!(should_timeout, Either::Second(_)));
    }

    #[test]
    async fn read_resolves_when_buffer_is_full(mut ctx: Context) {
        let mut data = [0u8; 8];

        ctx.rx
            .apply_config(&uart::Config::default().with_rx(RxConfig::default().with_timeout_none()))
            .unwrap();
        // Default RX FIFO threshold is higher than 8 bytes, so we should be able to
        // read all 8 bytes in one go.
        let (read, _) = join(ctx.rx.read_async(&mut data), async {
            ctx.tx.write_async(&[1, 2, 3, 4, 5, 6, 7, 8]).await.unwrap();
            ctx.tx.flush_async().await.unwrap();
        })
        .await;

        assert_eq!(read, Ok(8));
        assert_eq!(&data, &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    async fn cancelling_read_does_not_drop_data(mut ctx: Context) {
        let mut data = [0u8; 16];

        let should_be_tx = select(ctx.rx.read_async(&mut data), async {
            ctx.tx.write_async(&[1, 2, 3]).await.unwrap();
            ctx.tx.flush_async().await.unwrap();
        })
        .await;

        // Default RX FIFO threshold should be more than 3 bytes, so we don't expect
        // read to become ready.
        assert!(matches!(should_be_tx, Either::Second(_)));

        let read = ctx.rx.read_async(&mut data).await;

        assert_eq!(read, Ok(3));
        assert_eq!(&data[..3], &[1, 2, 3]);
    }
}

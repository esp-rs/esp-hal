//! UART Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use hil_test as _;

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Blocking,
        delay::Delay,
        gpio::{AnyPin, Pin},
        time::Duration,
        uart::{self, ClockSource, Uart},
    };

    struct Context {
        uart0: Uart<'static, Blocking>,
        uart1: Uart<'static, Blocking>,
        rx: AnyPin<'static>,
        tx: AnyPin<'static>,
        rts: AnyPin<'static>,
        delay: Delay,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let (rx, tx) = hil_test::common_test_pins!(peripherals);
        let rts = hil_test::unconnected_pin!(peripherals);

        let delay = Delay::new();

        let uart0 = Uart::new(peripherals.UART0, uart::Config::default()).unwrap();
        let uart1 = Uart::new(peripherals.UART1, uart::Config::default()).unwrap();

        Context {
            uart0,
            uart1,
            rx: rx.degrade(),
            tx: tx.degrade(),
            rts: rts.degrade(),
            delay: delay,
        }
    }

    #[test]
    fn test_send_receive(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        uart.write(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        uart.read(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);
    }

    #[test]
    fn flush_waits_for_data_to_be_transmitted(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        assert!(uart.write_ready());
        assert!(!uart.read_ready());

        let bauds = [1000, 5000000];
        for baud in bauds {
            uart.apply_config(&uart::Config::default().with_baudrate(baud))
                .unwrap();
            for i in 0..10 {
                let mut byte = [0u8; 1];
                uart.write(&[i as u8]).unwrap();
                uart.flush().unwrap();

                assert!(uart.write_ready());
                assert!(uart.read_ready());

                let read = uart.read_buffered(&mut byte).unwrap();

                assert!(!uart.read_ready());
                assert_eq!(read, 1, "Baud rate {}, iteration {}", baud, i);
                assert_eq!(byte[0], i as u8, "Baud rate {}, iteration {}", baud, i);
            }
        }
    }

    #[test]
    fn test_different_tolerance(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        let configs = [
            uart::Config::default()
                .with_baudrate(19_200)
                .with_baudrate_tolerance(uart::BaudrateTolerance::Exact),
            uart::Config::default()
                .with_baudrate(9600)
                .with_baudrate_tolerance(uart::BaudrateTolerance::ErrorPercent(10)),
        ];

        for config in configs {
            uart.apply_config(&config).unwrap();

            uart.write(&[0x42]).unwrap();
            let mut byte = [0u8; 1];
            uart.read(&mut byte).unwrap();
            assert_eq!(byte[0], 0x42);
        }
    }

    #[test]
    fn test_hw_flow_control(ctx: Context) {
        let (rts_input, rts_output) = unsafe { ctx.rts.split() };
        let mut uart = ctx
            .uart1
            .with_tx(ctx.tx)
            .with_rx(ctx.rx)
            .with_rts(rts_output);

        uart.apply_config(&uart::Config::default().with_hw_flow_ctrl(
            esp_hal::uart::HwFlowControl {
                cts: uart::CtsConfig::Disabled,
                rts: uart::RtsConfig::Enabled(4),
            },
        ))
        .unwrap();

        let data: [u8; 10] = [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9];

        // This will make RTS pin go high, as we write more than the RTS threshold.
        uart.write(&data).unwrap();
        // Required, otherwise we'll run into a timing issue and won't be able to
        // determine that the RTS is at a high level.
        ctx.delay.delay_millis(1);

        // RTS pin should go high when the threshold is exceeded.
        assert_eq!(rts_input.is_input_high(), true);
    }

    #[test]
    fn test_send_receive_buffer(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        const BUF_SIZE: usize = 128; // UART_FIFO_SIZE

        let data = [13; BUF_SIZE];
        let written = uart.write(&data).unwrap();
        assert_eq!(written, BUF_SIZE);

        // Calls to read may not fill the buffer, wait until read returns 0
        let mut buffer = [0; BUF_SIZE];
        embedded_io::Read::read_exact(&mut uart, &mut buffer).unwrap();

        assert_eq!(data, buffer);
    }

    #[test]
    fn test_send_receive_different_baud_rates_and_clock_sources(ctx: Context) {
        let mut uart = ctx.uart1.with_tx(ctx.tx).with_rx(ctx.rx);

        // The default baud rate for the UART is 115,200, so we will try to
        // send/receive with some other common baud rates to ensure this is
        // working as expected. We will also using different clock sources
        // while we're at it.

        cfg_if::cfg_if! {
            if #[cfg(esp32c2)] {
                let fastest_clock_source = ClockSource::PllF40m;
            } else if #[cfg(any(esp32c5, esp32c6))] {
                let fastest_clock_source = ClockSource::PllF80m;
            } else if #[cfg(esp32h2)] {
                let fastest_clock_source = ClockSource::PllF48m;
            } else {
                let fastest_clock_source = ClockSource::Apb;
            }
        }

        let configs = [
            #[cfg(not(soc_has_clock_node_ref_tick))]
            (9600, ClockSource::RcFast),
            #[cfg(not(soc_has_clock_node_ref_tick))]
            (19_200, ClockSource::Xtal),
            #[cfg(soc_has_clock_node_ref_tick)]
            (9600, ClockSource::RefTick),
            (921_600, fastest_clock_source),
            (2_000_000, fastest_clock_source),
            (3_500_000, fastest_clock_source),
            (5_000_000, fastest_clock_source),
        ];

        // TODO: we need a way to verify these baud rates are actually what we want.

        let mut byte_to_write = 0xA5;
        for (baudrate, clock_source) in configs {
            uart.apply_config(
                &uart::Config::default()
                    .with_baudrate(baudrate)
                    .with_clock_source(clock_source),
            )
            .unwrap();
            uart.write(&[byte_to_write]).unwrap();
            let mut byte = [0u8; 1];
            uart.read(&mut byte).unwrap();

            assert_eq!(byte[0], byte_to_write);
            byte_to_write = !byte_to_write;
        }
    }

    #[test]
    fn test_send_receive_inverted(ctx: Context) {
        let mut uart = ctx
            .uart1
            .with_tx(ctx.tx.into_output_signal().with_output_inverter(true))
            .with_rx(unsafe { ctx.rx.into_input_signal() }.with_input_inverter(true));

        uart.write(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        uart.read(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);
    }

    #[test]
    fn test_split_send_receive(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        let byte = [0x42];

        tx.flush().unwrap();
        tx.write(&byte).unwrap();
        let mut buf = [0u8; 1];
        rx.read(&mut buf).unwrap();

        assert_eq!(buf[0], 0x42);
    }

    #[test]
    fn test_split_send_receive_bytes(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        let bytes = [0x42, 0x43, 0x44];
        let mut buf = [0u8; 3];

        tx.flush().unwrap();
        tx.write(&bytes).unwrap();

        embedded_io::Read::read_exact(&mut rx, &mut buf).unwrap();

        assert_eq!(buf, bytes);
    }

    #[test]
    fn test_break_detection(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        rx.enable_break_detection();
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));
    }

    #[test]
    fn test_break_detection_no_break(ctx: Context) {
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        assert!(!rx.wait_for_break_with_timeout(Duration::from_millis(100)));
    }

    #[test]
    fn test_break_detection_multiple(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        rx.enable_break_detection();
        for _ in 0..3 {
            tx.send_break(100);
            assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));
        }
    }

    #[test]
    fn test_break_detection_interleaved(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        rx.enable_break_detection();

        // Test 1: Send break, expect detection
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));

        // Test 2: Don't send break, expect timeout
        assert!(!rx.wait_for_break_with_timeout(Duration::from_millis(100)));

        // Test 3: Send break again, expect detection
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));

        // Test 4: Don't send break, expect timeout again
        assert!(!rx.wait_for_break_with_timeout(Duration::from_millis(100)));

        // Test 5: Final break detection, expect detection
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));
    }

    #[test]
    fn test_break_detection_with_data(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        rx.enable_break_detection();

        // Test 1: Send break, expect detection
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));

        // Test 2: Send normal data (should not trigger break detection)
        tx.write(&[0x42, 0x43, 0x44]).unwrap();
        tx.flush().unwrap();
        let mut buf = [0u8; 3];
        rx.read(&mut buf).unwrap();
        assert_eq!(buf, [0x42, 0x43, 0x44]);

        // Test 3: Verify no false break detection after data
        assert!(!rx.wait_for_break_with_timeout(Duration::from_millis(100)));

        // Test 4: Send break after data, expect detection
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));

        // Test 5: Send more data
        tx.write(&[0xAA, 0xBB]).unwrap();
        tx.flush().unwrap();
        let mut buf = [0u8; 2];
        rx.read(&mut buf).unwrap();
        assert_eq!(buf, [0xAA, 0xBB]);

        // Test 6: Don't send break, expect timeout
        assert!(!rx.wait_for_break_with_timeout(Duration::from_millis(100)));

        // Test 7: Final break detection
        tx.send_break(100);
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));
    }

    #[test]
    fn test_break_detection_preserves_data(ctx: Context) {
        let mut tx = ctx.uart0.split().1.with_tx(ctx.tx);
        let mut rx = ctx.uart1.split().0.with_rx(ctx.rx);

        rx.enable_break_detection();

        // Send data, flush to ensure transmission, then send break
        tx.write(&[0x01, 0x02, 0x03, 0x04]).unwrap();
        tx.flush().unwrap();
        tx.send_break(100);

        // Detect the break
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));

        // Data should still be intact and readable after break detection
        let mut buf = [0u8; 4];
        rx.read(&mut buf).unwrap();
        assert_eq!(buf, [0x01, 0x02, 0x03, 0x04]);

        // Send more data after break
        tx.write(&[0xAA, 0xBB, 0xCC]).unwrap();
        tx.flush().unwrap();

        // Send another break
        tx.send_break(100);

        // Detect second break
        assert!(rx.wait_for_break_with_timeout(Duration::from_secs(1)));

        // Second batch of data should also be intact
        let mut buf = [0u8; 3];
        rx.read(&mut buf).unwrap();
        assert_eq!(buf, [0xAA, 0xBB, 0xCC]);
    }
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod async_tests {
    use core::fmt::Write;

    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    use esp_hal::{
        Async,
        Blocking,
        interrupt::{
            Priority,
            software::{SoftwareInterrupt, SoftwareInterruptControl},
        },
        timer::timg::TimerGroup,
        uart::{self, Uart, UartRx},
    };
    use esp_rtos::embassy::InterruptExecutor;
    use hil_test::mk_static;

    struct Context {
        interrupt: SoftwareInterrupt<'static, 1>,
        uart: Uart<'static, Async>,
    }

    const LONG_TEST_STRING: &str = "This is a very long string that will be printed to the serial console. First line of the string. Second line of the string. Third line of the string. Fourth line of the string. Fifth line of the string. Sixth line of the string.";

    #[embassy_executor::task]
    async fn long_string_reader(
        uart: UartRx<'static, Blocking>,
        signal: &'static Signal<CriticalSectionRawMutex, ()>,
    ) {
        let mut uart = uart.into_async();
        let mut received = 0;

        let mut buf = [0u8; 128];
        loop {
            let len = uart.read_async(&mut buf[..]).await.unwrap();
            received += len;
            assert!(received <= LONG_TEST_STRING.len());
            if received == LONG_TEST_STRING.len() {
                break;
            }
        }
        signal.signal(());
    }
    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let uart = Uart::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx)
            .into_async();

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        Context {
            interrupt: sw_int.software_interrupt1,
            uart,
        }
    }

    #[test]
    async fn test_send_receive(mut ctx: Context) {
        const SEND: &[u8] = b"Hello ESP32";
        let mut buf = [0u8; SEND.len()];

        ctx.uart.flush_async().await.unwrap();
        ctx.uart.write_async(&SEND).await.unwrap();
        ctx.uart.flush_async().await.unwrap();

        ctx.uart.read_async(&mut buf[..]).await.unwrap();
        assert_eq!(&buf[..], SEND);
    }

    #[test]
    async fn test_long_strings(ctx: Context) {
        // This is a regression test for https://github.com/esp-rs/esp-hal/issues/3450
        // The issue was that the printed string is longer than the UART buffer so the
        // end was lost. We can't test the fix in blocking code, only on
        // dual-cores, but a preemptive interrupt executor should work on all
        // chips.

        let (rx, mut tx) = ctx.uart.into_blocking().split();

        let interrupt_executor =
            mk_static!(InterruptExecutor<1>, InterruptExecutor::new(ctx.interrupt));

        let signal = &*mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        let spawner = interrupt_executor.start(Priority::Priority3);
        spawner.must_spawn(long_string_reader(rx, signal));

        tx.write_str(LONG_TEST_STRING).unwrap();

        // Wait for the signal to be sent
        signal.wait().await;
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod tx_rx_regression {
    use esp_hal::{
        gpio::Flex,
        uart::{self, UartRx, UartTx},
    };
    #[test]
    fn test_that_creating_tx_does_not_cause_a_pulse() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let mut rx = UartRx::new(peripherals.UART1, uart::Config::default())
            .unwrap()
            .with_rx(rx);

        // Start from a low level to verify that UartTx sets the level high initially,
        // but don't enable output otherwise we actually pull down against RX's
        // pullup resistor.
        let mut tx = Flex::new(tx);
        tx.set_low();

        // set up TX and send a byte
        let mut tx = UartTx::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx);

        tx.flush().unwrap();
        tx.write(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        rx.read(&mut byte).unwrap();

        assert_eq!(byte[0], 0x42);
    }
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod async_tx_rx {
    use embassy_futures::{
        join::join,
        select::{Either, select},
    };
    use embassy_time::{Duration, Timer};
    use embedded_io_async::Write;
    use esp_hal::{
        Async,
        interrupt::software::SoftwareInterruptControl,
        timer::timg::TimerGroup,
        uart::{self, RxConfig, RxError, UartRx, UartTx},
    };

    struct Context {
        rx: UartRx<'static, Async>,
        tx: UartTx<'static, Async>,
    }
    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

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
            // bytes by default. Allow for inequality in case processing is held up a bit.
            let read_count = ctx.rx.read_async(&mut read).await.unwrap();
            assert!(read_count >= 120);

            ctx.rx
                .read_exact_async(&mut read[read_count..])
                .await
                .unwrap();
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

#[cfg(uhci_driver_supported)]
#[embedded_test::tests(default_timeout = 5, executor = hil_test::Executor::new())]
mod uhci {
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
        interrupt::software::SoftwareInterruptControl,
        peripherals::Peripherals,
        timer::timg::TimerGroup,
        uart::{self, Uart, uhci::Uhci},
    };

    struct Context {
        dma_rx: DmaRxBuf,
        dma_tx: DmaTxBuf,
        peripherals: Peripherals,
    }

    const LONG_TEST_STRING: &str = "Loremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.InLoremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,bibendumeusem.Donecaccumsanrisusnibh,etefficiturnisivehiculatempus.Etiamegestasenimatduieleifendmaximus.Nuncinsemperest.Etiamvelodioultrices,interdumeratsed,dignissimmetus.Phasellusexleo,eleifendquisexid,laciniavenenatisneque.Sednuncdiam,molestieveltinciduntnec,ornareetnisi.Maecenasetmolestietortor.Nullaeupulvinarquam.Aeneanmolestieliberoquistortorviverralobortis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunecenim.Proinvenenatistortorveltristiquealiquam.Utelementumtellusligula,velauctorexfermentuma.Vestibulummaximusanteinvulputateornare.Sedquisnislaligulaporttitorfacilisismattissedmi.Crasconsecteturexegetsagittisfeugiat.Invenenatisminectinciduntaliquet.Sedcommodonecorciidvenenatis.Vestibulumanteipsumprimisinfaucibusorciluctusetultricesposuerecubiliacurae;Phasellusinterdumorcrtis.Praesentlaoreetlectusattinciduntscelerisque.Suspendisseegeterateleifend,posuerenuncvenenatis,faucibusdolor.Nuncvitaeluctusmetus.Nullamultriciesarcuvitaeestfermentumeleifend.Suspendisselaoreetmaximuslacus,utlaoreetnisiiaculisvitae.Nullamscelerisqueporttitorpulvinar.Intinciduntipsummauris,velaliquetmetusdictumut.Nunceratelit,suscipitacnisiac,volutpatporttitormauris.Aliquampretiumnisidiam,molestietemporlacusplaceratid.Mauristinciduntmattisturpis,velconvallisurnatempusnon.Integermattismetusnoneuismodcursus.Namideratetmassapretiumfinibus.Praesentfermentumnuncurna,quissagittismaurisimperdieteu.Inefficituraliquamdui.Phasellussempermaurisacconvallismollis.Suspendisseintellusanuncvariusiaculisutegetlibero.Inmalesuada,nislquisconsecteturposuere,nullaipsumfringillaeros,egetrhoncussapienarcunece";
    // const MEDIUM_TEST_STRING: &str =
    // "Loremipsumdolorsitamet,consecteturadipiscingelit.Suspendissemetusnisl,pretiumsedeuismodeget,
    // bibendumeusem.Donecaccumsanrisusnibh";
    const SHORT_TEST_STRING: &str = "Hello ESP32";
    const DMA_BUFFER_SIZE: u16 = 4092;
    const LOOP_COUNT: u32 = 5;
    const BAUDRATE: u32 = 921600;
    // const BAUDRATES: &[u32] = &[9600, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 460800,
    // 576000, 921600];

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers!(DMA_BUFFER_SIZE as usize);
        let dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        Context {
            dma_rx,
            dma_tx,
            peripherals,
        }
    }

    #[test]
    fn test_send_receive(ctx: Context) {
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..SHORT_TEST_STRING.len()]
                .copy_from_slice(&SHORT_TEST_STRING.as_bytes());
            dma_tx.set_length(SHORT_TEST_STRING.len());

            let transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                SHORT_TEST_STRING.as_bytes()
            );

            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    #[test]
    fn test_long_strings(ctx: Context) {
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..LONG_TEST_STRING.len()]
                .copy_from_slice(&LONG_TEST_STRING.as_bytes());
            dma_tx.set_length(LONG_TEST_STRING.len());

            let transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                LONG_TEST_STRING.as_bytes()
            );

            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    #[test]
    async fn test_send_receive_async(ctx: Context) {
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let uhci = uhci.into_async();
        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..SHORT_TEST_STRING.len()]
                .copy_from_slice(&SHORT_TEST_STRING.as_bytes());
            dma_tx.set_length(SHORT_TEST_STRING.len());

            let mut transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let mut transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            transfer_tx.wait_for_done().await;
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            transfer_rx.wait_for_done().await;
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                SHORT_TEST_STRING.as_bytes()
            );
            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    #[test]
    async fn test_long_strings_async(ctx: Context) {
        let sw_int = SoftwareInterruptControl::new(ctx.peripherals.SW_INTERRUPT);

        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

        let (rx, tx) = hil_test::common_test_pins!(ctx.peripherals);
        let uart = Uart::new(ctx.peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);
        let mut uhci = Uhci::new(uart, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
        uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
            .unwrap();
        uhci.apply_tx_config(&uart::uhci::TxConfig::default())
            .unwrap();
        uhci.set_uart_config(&uart::Config::default().with_baudrate(BAUDRATE))
            .unwrap();

        let uhci = uhci.into_async();
        let (uhci_rx, uhci_tx) = uhci.split();
        let mut uhci_rx_opt = Some(uhci_rx);
        let mut uhci_tx_opt = Some(uhci_tx);
        let mut dma_rx_opt = Some(ctx.dma_rx);
        let mut dma_tx_opt = Some(ctx.dma_tx);

        for _ in 0..LOOP_COUNT {
            let uhci_rx = uhci_rx_opt.take().unwrap();
            let uhci_tx = uhci_tx_opt.take().unwrap();
            let dma_rx = dma_rx_opt.take().unwrap();
            let mut dma_tx = dma_tx_opt.take().unwrap();

            dma_tx.as_mut_slice()[0..LONG_TEST_STRING.len()]
                .copy_from_slice(&LONG_TEST_STRING.as_bytes());
            dma_tx.set_length(LONG_TEST_STRING.len());

            let mut transfer_rx = uhci_rx
                .read(dma_rx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            let mut transfer_tx = uhci_tx
                .write(dma_tx)
                .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
            transfer_tx.wait_for_done().await;
            let (res, uhci_tx, dma_tx) = transfer_tx.wait();
            res.unwrap();
            transfer_rx.wait_for_done().await;
            let (res, uhci_rx, dma_rx) = transfer_rx.wait();
            res.unwrap();

            assert_eq!(
                &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
                LONG_TEST_STRING.as_bytes()
            );
            uhci_rx_opt = Some(uhci_rx);
            uhci_tx_opt = Some(uhci_tx);
            dma_rx_opt = Some(dma_rx);
            dma_tx_opt = Some(dma_tx);
        }
    }

    // Will be re-enabled and fully implemented once changing uart baudrate after split is supported
    // #[test]
    // async fn test_baudrate_change_rx(ctx: Context) {
    // let (rx1, tx1) = hil_test::common_test_pins!(ctx.peripherals);
    // let (rx2, tx2) = hil_test::i2c_pins!(ctx.peripherals);
    //
    // let uart0 = Uart::new(ctx.peripherals.UART0, uart::Config::default())
    // .unwrap()
    // .with_tx(tx2)
    // .with_rx(rx1);
    // let mut uhci = Uhci::new(uart0, ctx.peripherals.UHCI0, ctx.peripherals.DMA_CH0);
    // uhci.apply_rx_config(&uart::uhci::RxConfig::default().with_chunk_limit(DMA_BUFFER_SIZE))
    // .unwrap();
    // uhci.apply_tx_config(&uart::uhci::TxConfig::default())
    // .unwrap();
    //
    // let mut uart1 = Uart::new(ctx.peripherals.UART1, uart::Config::default())
    // .unwrap()
    // .with_tx(tx1)
    // .with_rx(rx2);
    //
    // let uhci = uhci.into_async();
    //
    // let (uhci_rx, uhci_tx) = uhci.split();
    // let mut uhci_rx_opt = Some(uhci_rx);
    // let mut uhci_tx_opt = Some(uhci_tx);
    // let mut dma_rx_opt = Some(ctx.dma_rx);
    // for baudrate in BAUDRATES {
    // let mut uhci_rx = uhci_rx_opt.take().unwrap();
    // let mut uhci_tx = uhci_tx_opt.take().unwrap();
    // let dma_rx = dma_rx_opt.take().unwrap();
    //
    // let config = &uart::Config::default().with_baudrate(*baudrate);
    // uart1.apply_config(config).unwrap();
    // THE CONFIG IS A LIE
    // :(
    // uhci_rx.uart_rx.apply_config(config).unwrap();
    // uhci_tx.uart_tx.apply_config(config).unwrap();
    // let mut transfer_rx = uhci_rx
    // .read(dma_rx)
    // .unwrap_or_else(|x| panic!("Something went horribly wrong: {:?}", x.0));
    // uart1.write(MEDIUM_TEST_STRING.as_bytes()).unwrap();
    // transfer_rx.wait_for_done().await;
    // let (res, uhci_rx, dma_rx) = transfer_rx.wait();
    // res.unwrap();
    // assert_eq!(
    // &dma_rx.as_slice()[0..dma_rx.number_of_received_bytes()],
    // MEDIUM_TEST_STRING.as_bytes()
    // );
    // uhci_rx_opt = Some(uhci_rx);
    // uhci_tx_opt = Some(uhci_tx);
    // dma_rx_opt = Some(dma_rx);
    // }
    // }
    //
    // #[test]
    // async fn test_baudrate_change_tx(ctx: Context) {
    // return;
    // }
}

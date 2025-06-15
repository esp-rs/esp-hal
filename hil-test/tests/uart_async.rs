//! UART Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable embassy

#![no_std]
#![no_main]

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::{
    Async,
    Blocking,
    interrupt::{
        Priority,
        software::{SoftwareInterrupt, SoftwareInterruptControl},
    },
    uart::{self, Uart, UartRx},
};
use esp_hal_embassy::InterruptExecutor;
use hil_test::mk_static;

esp_bootloader_esp_idf::esp_app_desc!();

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

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {

    use core::fmt::Write;

    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let uart = Uart::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx)
            .into_async();

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        Context {
            interrupt: sw_ints.software_interrupt1,
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

//! UART Break Detection test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy

#![no_std]
#![no_main]

use esp_hal::{
    uart::{Uart, Config as UartConfig,},
    Blocking,
};
use hil_test as _;

struct Context {
    uart: Uart<'static, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (_, pin) = hil_test::common_test_pins!(peripherals);
        let (rx, tx) = pin.split();
        let uart = Uart::new(peripherals.UART1, UartConfig::default())
            .expect("Failed to initialize UART")
            .with_rx(rx)
            .with_tx(tx);

        Context { uart }
    }

    #[test]
    fn test_wait_for_break_blocking(mut ctx: Context) {
        // TODO: Send (or simulate) a break signal, otherwise this will fail via timeout
        ctx.uart.wait_for_break();
    }

    #[test]
    async fn test_wait_for_break_async(ctx: Context) {
        // TODO: Send (or simulate) a break signal, otherwise this will fail via timeout
        ctx.uart.into_async().wait_for_break_async().await;
    }
}

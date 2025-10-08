//! UART Break Detection test
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    uart::{Config as UartConfig, Uart},
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

        let (rx, tx) = hil_test::common_test_pins!(peripherals);
        let uart = Uart::new(peripherals.UART1, UartConfig::default())
            .expect("Failed to initialize UART")
            .with_rx(rx)
            .with_tx(tx);

        Context { uart }
    }

    #[test]
    async fn test_break_detection(mut ctx: Context) {
        ctx.uart.send_break(1);
        ctx.uart.into_async().wait_for_break_async().await;
        // should exit with break detected before timeout
    }
}

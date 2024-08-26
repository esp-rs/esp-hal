//! UART TX/RX Async Test
//!
//! Folowing pins are used:
//! TX    GPIP2
//! RX    GPIO3
//!
//! Connect TX (GPIO2) and RX (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use esp_hal::{
    gpio::Io,
    peripherals::{UART0, UART1},
    prelude::*,
    uart::{UartRx, UartTx},
    Async,
};
use hil_test as _;

struct Context {
    tx: UartTx<'static, UART0, Async>,
    rx: UartRx<'static, UART1, Async>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    async fn init() -> Context {
        let (peripherals, clocks) = esp_hal::init(Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let tx = UartTx::new_async(peripherals.UART0, &clocks, io.pins.gpio2).unwrap();
        let rx = UartRx::new_async(peripherals.UART1, &clocks, io.pins.gpio3).unwrap();

        Context { tx, rx }
    }

    #[test]
    #[timeout(3)]
    async fn test_send_receive(mut ctx: Context) {
        let byte = [0x42];
        let mut read = [0u8; 1];

        ctx.tx.flush_async().await.unwrap();
        ctx.tx.write_async(&byte).await.unwrap();
        let _ = ctx.rx.read_async(&mut read).await;

        assert_eq!(read, byte);
    }
}

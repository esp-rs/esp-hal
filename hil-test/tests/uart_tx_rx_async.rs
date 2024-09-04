//! UART TX/RX Async Test
//!
//! Folowing pins are used:
//! TX    GPIO2 / GPIO9  (esp32s2 / esp32s3) / GPIO26 (esp32)
//! RX    GPIO3 / GPIO10 (esp32s2 / esp32s3) / GPIO27 (esp32)
//!
//! Connect TX and RX pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use esp_hal::{
    gpio::Io,
    peripherals::{UART0, UART1},
    uart::{UartRx, UartTx},
    Async,
};
use hil_test as _;

struct Context {
    rx: UartRx<'static, UART1, Async>,
    tx: UartTx<'static, UART0, Async>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (tx, rx) = hil_test::common_test_pins!(io);

        let tx = UartTx::new_async(peripherals.UART0, tx).unwrap();
        let rx = UartRx::new_async(peripherals.UART1, rx).unwrap();

        Context { rx, tx }
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

//! UART TX/RX Async Test
//!
//! Folowing pins are used:
//! TX    GPIP2
//! RX    GPIO4
//!
//! Connect TX (GPIO2) and RX (GPIO4) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::Io,
    peripherals::{Peripherals, UART0, UART1},
    system::SystemControl,
    uart::{UartRx, UartTx},
    Async,
};

struct Context {
    tx: UartTx<'static, UART0, Async>,
    rx: UartRx<'static, UART1, Async>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let _system = SystemControl::new(peripherals.SYSTEM);

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let tx = UartTx::new_async(peripherals.UART0, io.pins.gpio2);
        let rx = UartRx::new_async(peripherals.UART1, io.pins.gpio4);

        Context { tx, rx }
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal::embassy::executor::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    async fn init() -> Context {
        Context::init()
    }

    #[test]
    #[timeout(3)]
    async fn test_send_receive(mut ctx: Context) {
        let byte = [0x42];
        let mut read = [0u8; 1];

        ctx.tx.flush_async().await.unwrap();
        ctx.tx.write_async(&byte).await.unwrap();
        let read = ctx.rx.read_async(&mut read).await;

        assert_eq!(read, Ok(0x42));
    }
}

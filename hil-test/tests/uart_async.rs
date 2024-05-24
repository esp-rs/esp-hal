//! UART Test
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
    clock::ClockControl,
    gpio::Io,
    peripherals::{Peripherals, UART0},
    system::SystemControl,
    uart::{config::Config, TxRxPins, Uart, UartRx, UartTx},
    Async,
};

struct Context {
    tx: UartTx<'static, UART0, Async>,
    rx: UartRx<'static, UART0, Async>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pins = TxRxPins::new_tx_rx(io.pins.gpio2, io.pins.gpio4);

        let uart =
            Uart::new_async_with_config(peripherals.UART0, Config::default(), Some(pins), &clocks);
        let (tx, rx) = uart.split();

        Context { rx, tx }
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::executor::Executor::new())]
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
        const SEND: &[u8] = &*b"Hello ESP32";
        let mut buf = [0u8; SEND.len()];

        // Drain the FIFO to clear previous message:
        ctx.tx.flush_async().await.unwrap();
        while ctx.rx.drain_fifo(&mut buf[..]) > 0 {}

        ctx.tx.write_async(&SEND).await.unwrap();
        ctx.tx.flush_async().await.unwrap();

        ctx.rx.read_async(&mut buf[..]).await.unwrap();
        assert_eq!(&buf[..], SEND);
    }
}

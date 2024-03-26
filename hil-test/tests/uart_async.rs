//! UART Test
//!
//! Folowing pins are used:
//! TX    GPIP2
//! RX    GPIO4
//!
//! Connect TX (GPIO2) and RX (GPIO4) pins.

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::{Peripherals, UART0},
    prelude::*,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
        Uart,
        UartRx,
        UartTx,
    },
    Async,
};

struct Context {
    tx: UartTx<'static, UART0, Async>,
    rx: UartRx<'static, UART0, Async>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        let pins = TxRxPins::new_tx_rx(
            io.pins.gpio2.into_push_pull_output(),
            io.pins.gpio4.into_floating_input(),
        );
        let config = Config {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        };

        let uart = Uart::new_async_with_config(peripherals.UART0, config, Some(pins), &clocks);
        let (tx, rx) = uart.split();

        Context { rx, tx }
    }
}

#[cfg(test)]
#[embedded_test::tests]
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

//! UART TX/RX Test
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
    prelude::*,
    system::SystemControl,
    uart::{UartRx, UartTx},
    Blocking,
};
use nb::block;

struct Context {
    tx: UartTx<'static, UART0, Blocking>,
    rx: UartRx<'static, UART1, Blocking>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let _system = SystemControl::new(peripherals.SYSTEM);

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let tx = UartTx::new(peripherals.UART0, io.pins.gpio2);
        let rx = UartRx::new(peripherals.UART1, io.pins.gpio4);

        Context { tx, rx }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        Context::init()
    }

    #[test]
    #[timeout(3)]
    fn test_send_receive(mut ctx: Context) {
        let byte = [0x42];

        ctx.tx.flush_tx().unwrap();
        ctx.tx.write_bytes(&byte).unwrap();
        let read = block!(ctx.rx.read_byte());

        assert_eq!(read, Ok(0x42));
    }
}

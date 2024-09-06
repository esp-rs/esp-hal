//! UART TX/RX Test
//!
//! Folowing pins are used:
//! TX    GPIO2 / GPIO9  (esp32s2 / esp32s3) / GPIO26 (esp32)
//! RX    GPIO3 / GPIO10 (esp32s2 / esp32s3) / GPIO27 (esp32)
//!
//! Connect TX and RX pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    gpio::Io,
    peripherals::{UART0, UART1},
    prelude::*,
    uart::{UartRx, UartTx},
    Blocking,
};
use hil_test as _;
use nb::block;

struct Context {
    rx: UartRx<'static, UART1, Blocking>,
    tx: UartTx<'static, UART0, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (rx, tx) = hil_test::common_test_pins!(io);

        let tx = UartTx::new(peripherals.UART0, tx).unwrap();
        let rx = UartRx::new(peripherals.UART1, rx).unwrap();

        Context { rx, tx }
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

    #[test]
    #[timeout(3)]
    fn test_send_receive_bytes(mut ctx: Context) {
        let bytes = [0x42, 0x43, 0x44];
        let mut buf = [0u8; 3];

        ctx.tx.flush_tx().unwrap();
        ctx.tx.write_bytes(&bytes).unwrap();

        ctx.rx.read_bytes(&mut buf).unwrap();

        assert_eq!(buf, bytes);
    }
}

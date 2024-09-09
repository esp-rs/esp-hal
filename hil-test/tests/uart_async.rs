//! UART Test
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

use esp_hal::{gpio::Io, peripherals::UART0, uart::Uart, Async};
use hil_test as _;

struct Context {
    uart: Uart<'static, UART0, Async>,
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

        let (rx, tx) = hil_test::common_test_pins!(io);

        let uart = Uart::new_async(peripherals.UART0, rx, tx).unwrap();

        Context { uart }
    }

    #[test]
    #[timeout(3)]
    async fn test_send_receive(mut ctx: Context) {
        const SEND: &[u8] = &*b"Hello ESP32";
        let mut buf = [0u8; SEND.len()];

        ctx.uart.flush_async().await.unwrap();
        ctx.uart.write_async(&SEND).await.unwrap();
        ctx.uart.flush_async().await.unwrap();

        ctx.uart.read_async(&mut buf[..]).await.unwrap();
        assert_eq!(&buf[..], SEND);
    }
}

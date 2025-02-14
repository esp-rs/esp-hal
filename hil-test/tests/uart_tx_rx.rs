//! UART TX/RX Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    uart::{self, UartRx, UartTx},
    Blocking,
};
use hil_test as _;

struct Context {
    rx: UartRx<'static, Blocking>,
    tx: UartTx<'static, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let tx = UartTx::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx);
        let rx = UartRx::new(peripherals.UART1, uart::Config::default())
            .unwrap()
            .with_rx(rx);

        Context { rx, tx }
    }

    #[test]
    fn test_send_receive(mut ctx: Context) {
        let byte = [0x42];

        ctx.tx.flush();
        ctx.tx.write(&byte).unwrap();
        let mut buf = [0u8; 1];
        ctx.rx.read(&mut buf).unwrap();

        assert_eq!(buf[0], 0x42);
    }

    #[test]
    fn test_send_receive_bytes(mut ctx: Context) {
        let bytes = [0x42, 0x43, 0x44];
        let mut buf = [0u8; 3];

        ctx.tx.flush();
        ctx.tx.write(&bytes).unwrap();

        ctx.rx.read(&mut buf).unwrap();

        assert_eq!(buf, bytes);
    }
}

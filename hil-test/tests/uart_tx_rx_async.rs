//! UART TX/RX Async Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use esp_hal::{
    gpio::Io,
    uart::{UartRx, UartTx},
    Async,
};
use hil_test as _;

struct Context {
    rx: UartRx<'static, Async>,
    tx: UartTx<'static, Async>,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (rx, tx) = hil_test::common_test_pins!(io);

        let tx = UartTx::new(peripherals.UART0, tx).unwrap().into_async();
        let rx = UartRx::new(peripherals.UART1, rx).unwrap().into_async();

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

//! UART Test
//!
//! Folowing pins are used:
//! TX    GPIP2
//! RX    GPIO3
//!
//! Connect TX (GPIO2) and RX (GPIO3) pins.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    peripherals::{Peripherals, UART0},
    system::SystemControl,
    uart::Uart,
    Async,
};
use hil_test as _;

struct Context {
    uart: Uart<'static, UART0, Async>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let uart =
            Uart::new_async(peripherals.UART0, &clocks, io.pins.gpio2, io.pins.gpio3).unwrap();

        Context { uart }
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
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

        ctx.uart.flush_async().await.unwrap();
        ctx.uart.write_async(&SEND).await.unwrap();
        ctx.uart.flush_async().await.unwrap();

        ctx.uart.read_async(&mut buf[..]).await.unwrap();
        assert_eq!(&buf[..], SEND);
    }
}

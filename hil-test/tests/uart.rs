//! UART Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    uart::{self, ClockSource, Uart},
    Blocking,
};
use hil_test as _;

struct Context {
    uart: Uart<'static, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let uart = Uart::new(peripherals.UART1, uart::Config::default())
            .unwrap()
            .with_tx(tx)
            .with_rx(rx);

        Context { uart }
    }

    #[test]
    fn test_send_receive(mut ctx: Context) {
        ctx.uart.write_bytes(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        ctx.uart.read_bytes(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);
    }

    #[test]
    fn test_different_tolerance(mut ctx: Context) {
        ctx.uart
            .apply_config(
                &uart::Config::default()
                    .with_baudrate(19_200)
                    .with_baudrate_tolerance(uart::BaudrateTolerance::Exact),
            )
            .unwrap();

        ctx.uart.write_bytes(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        ctx.uart.read_bytes(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);

        ctx.uart
            .apply_config(
                &uart::Config::default()
                    .with_baudrate(9600)
                    .with_baudrate_tolerance(uart::BaudrateTolerance::ErrorPercent(10)),
            )
            .unwrap();

        ctx.uart.write_bytes(&[0x42]).unwrap();
        let mut byte = [0u8; 1];
        ctx.uart.read_bytes(&mut byte).unwrap();
        assert_eq!(byte[0], 0x42);
    }

    #[test]
    fn test_send_receive_buffer(mut ctx: Context) {
        const BUF_SIZE: usize = 128; // UART_FIFO_SIZE

        let data = [13; BUF_SIZE];
        let written = ctx.uart.write_bytes(&data).unwrap();
        assert_eq!(written, BUF_SIZE);

        let mut buffer = [0; BUF_SIZE];

        ctx.uart.read_bytes(&mut buffer).unwrap();

        assert_eq!(data, buffer);
    }

    #[test]
    fn test_send_receive_different_baud_rates_and_clock_sources(mut ctx: Context) {
        // The default baud rate for the UART is 115,200, so we will try to
        // send/receive with some other common baud rates to ensure this is
        // working as expected. We will also using different clock sources
        // while we're at it.
        let configs = [
            #[cfg(not(any(esp32, esp32s2)))]
            (9600, ClockSource::RcFast),
            #[cfg(not(any(esp32, esp32s2)))]
            (19_200, ClockSource::Xtal),
            #[cfg(esp32s2)]
            (9600, ClockSource::RefTick),
            (921_600, ClockSource::Apb),
            (2_000_000, ClockSource::Apb),
        ];

        let mut byte_to_write = 0xA5;
        for (baudrate, clock_source) in configs {
            ctx.uart
                .apply_config(
                    &uart::Config::default()
                        .with_baudrate(baudrate)
                        .with_clock_source(clock_source),
                )
                .unwrap();
            ctx.uart.write_bytes(&[byte_to_write]).unwrap();
            let mut byte = [0u8; 1];
            ctx.uart.read_bytes(&mut byte).unwrap();

            assert_eq!(byte[0], byte_to_write);
            byte_to_write = !byte_to_write;
        }
    }
}

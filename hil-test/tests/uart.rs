//! UART Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal_02::serial::{Read, Write};
use esp_hal::{
    gpio::Io,
    peripherals::UART1,
    prelude::*,
    uart::{ClockSource, Uart},
    Blocking,
};
use hil_test as _;
use nb::block;

struct Context {
    uart: Uart<'static, UART1, Blocking>,
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

        let (_, pin) = hil_test::common_test_pins!(io);

        let uart = Uart::new(
            peripherals.UART1,
            pin.peripheral_input(),
            pin.into_peripheral_output(),
        )
        .unwrap();

        Context { uart }
    }

    #[test]
    #[timeout(3)]
    fn test_send_receive(mut ctx: Context) {
        ctx.uart.write(0x42).ok();
        let read = block!(ctx.uart.read());
        assert_eq!(read, Ok(0x42));
    }

    #[test]
    #[timeout(3)]
    fn test_send_receive_buffer(mut ctx: Context) {
        const BUF_SIZE: usize = 128; // UART_FIFO_SIZE

        let data = [13; BUF_SIZE];
        let written = ctx.uart.write_bytes(&data).unwrap();
        assert_eq!(written, BUF_SIZE);

        let mut buffer = [0; BUF_SIZE];
        let mut i = 0;

        while i < BUF_SIZE {
            match ctx.uart.read() {
                Ok(byte) => {
                    buffer[i] = byte;
                    i += 1;
                }
                Err(nb::Error::WouldBlock) => continue,
                Err(nb::Error::Other(_)) => panic!(),
            }
        }

        assert_eq!(data, buffer);
    }

    #[test]
    #[timeout(3)]
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
        ];

        let mut byte_to_write = 0xA5;
        for (baud, clock_source) in &configs {
            ctx.uart.change_baud(*baud, *clock_source);
            ctx.uart.write(byte_to_write).ok();
            let read = block!(ctx.uart.read());
            assert_eq!(read, Ok(byte_to_write));
            byte_to_write = !byte_to_write;
        }
    }
}

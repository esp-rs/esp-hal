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

use defmt_rtt as _;
use embedded_hal_02::serial::{Read, Write};
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    gpio::Io,
    peripherals::{Peripherals, UART1},
    prelude::*,
    system::SystemControl,
    uart::{ClockSource, Uart},
    Blocking,
};
use nb::block;

struct Context {
    clocks: Clocks<'static>,
    uart: Uart<'static, UART1, Blocking>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let uart = Uart::new(peripherals.UART1, &clocks, io.pins.gpio2, io.pins.gpio3).unwrap();

        Context { clocks, uart }
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

        #[cfg(not(feature = "esp32s2"))]
        {
            #[cfg(not(any(feature = "esp32", feature = "esp32c3", feature = "esp32c2")))]
            {
                // 9600 baud, RC FAST clock source:
                ctx.uart.change_baud(9600, ClockSource::RcFast, &ctx.clocks);
                ctx.uart.write(7).ok();
                let read = block!(ctx.uart.read());
                assert_eq!(read, Ok(7));
            }

            // 19,200 baud, XTAL clock source:
            #[cfg(not(feature = "esp32"))]
            {
                ctx.uart.change_baud(19_200, ClockSource::Xtal, &ctx.clocks);
                ctx.uart.write(55).ok();
                let read = block!(ctx.uart.read());
                assert_eq!(read, Ok(55));
            }

            // 921,600 baud, APB clock source:
            ctx.uart.change_baud(921_600, ClockSource::Apb, &ctx.clocks);
            ctx.uart.write(253).ok();
            let read = block!(ctx.uart.read());
            assert_eq!(read, Ok(253));
        }

        #[cfg(feature = "esp32s2")]
        {
            // 9600 baud, REF TICK clock source:
            ctx.uart
                .change_baud(9600, ClockSource::RefTick, &ctx.clocks);
            ctx.uart.write(7).ok();
            let read = block!(ctx.uart.read());
            assert_eq!(read, Ok(7));

            // 921,600 baud, APB clock source:
            ctx.uart.change_baud(921_600, ClockSource::Apb, &ctx.clocks);
            ctx.uart.write(253).ok();
            let read = block!(ctx.uart.read());
            assert_eq!(read, Ok(253));
        }
    }
}

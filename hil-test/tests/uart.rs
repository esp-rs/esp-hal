//! UART Test
//!
//! Folowing pins are used:
//! TX    GPIP9
//! RX    GPIO10
//!
//! Connect TX (GPI10) and RX (GPIO9) pins.

#![no_std]
#![no_main]

use hil_test::esp_hal::{
    clock::ClockControl,
    peripherals::{Peripherals, UART0},
    prelude::*,
    uart::{
        config::{Config, DataBits, Parity, StopBits},
        TxRxPins,
    },
    Uart, IO,
};
use nb::block;

struct Context {
    uart: Uart<'static, UART0>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        let pins = TxRxPins::new_tx_rx(
            io.pins.gpio10.into_push_pull_output(),
            io.pins.gpio9.into_floating_input(),
        );
        let config = Config {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        };

        let uart = Uart::new_with_config(peripherals.UART0, config, Some(pins), &clocks);

        Context { uart }
    }
}

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
}

//! TWAI test
//!
//! Folowing pins are used:
//! TX    GPIO2
//! RX    GPIO3
//!
//! Connect TX (GPIO2) and RX (GPIO3) pins.

//% CHIPS: esp32c3 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal_02::can::Frame;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    peripherals::{Peripherals, TWAI0},
    prelude::*,
    system::SystemControl,
    twai::{self, filter::SingleStandardFilter, EspTwaiFrame, StandardId, TwaiMode},
    Blocking,
};
use nb::block;

struct Context {
    twai: twai::Twai<'static, TWAI0, Blocking>,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let can_tx_pin = io.pins.gpio2;
        let can_rx_pin = io.pins.gpio3;

        const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B1000K;

        let mut config = twai::TwaiConfiguration::new(
            peripherals.TWAI0,
            can_tx_pin,
            can_rx_pin,
            &clocks,
            CAN_BAUDRATE,
            TwaiMode::SelfTest,
        );

        const FILTER: SingleStandardFilter =
            SingleStandardFilter::new(b"00000000000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
        config.set_filter(FILTER);

        let twai = config.start();

        Context { twai }
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
        let frame = EspTwaiFrame::new_self_reception(StandardId::ZERO.into(), &[1, 2, 3]).unwrap();
        block!(ctx.twai.transmit(&frame)).unwrap();

        let frame = block!(ctx.twai.receive()).unwrap();

        assert_eq!(frame.data(), &[1, 2, 3])
    }
}

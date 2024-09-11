//! TWAI test

//% CHIPS: esp32c3 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal_02::can::Frame;
use esp_hal::{
    gpio::Io,
    peripherals::TWAI0,
    prelude::*,
    twai::{self, filter::SingleStandardFilter, EspTwaiFrame, StandardId, TwaiMode},
    Blocking,
};
use hil_test as _;
use nb::block;

struct Context {
    twai: twai::Twai<'static, TWAI0, Blocking>,
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

        let (can_tx_pin, can_rx_pin) = hil_test::common_test_pins!(io);

        let mut config = twai::TwaiConfiguration::new(
            peripherals.TWAI0,
            can_rx_pin,
            can_tx_pin,
            twai::BaudRate::B1000K,
            TwaiMode::SelfTest,
        );

        const FILTER: SingleStandardFilter =
            SingleStandardFilter::new(b"00000000000", b"x", [b"xxxxxxxx", b"xxxxxxxx"]);
        config.set_filter(FILTER);

        let twai = config.start();

        Context { twai }
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

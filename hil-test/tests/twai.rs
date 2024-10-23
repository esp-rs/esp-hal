//! TWAI test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal_02::can::Frame;
use esp_hal::{
    gpio::Io,
    prelude::*,
    twai::{self, filter::SingleStandardFilter, EspTwaiFrame, StandardId, TwaiMode},
    Blocking,
};
use hil_test as _;
use nb::block;

struct Context {
    twai: twai::Twai<'static, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (loopback_pin, _) = hil_test::common_test_pins!(io);

        let mut config = twai::TwaiConfiguration::new(
            peripherals.TWAI0,
            loopback_pin.peripheral_input(),
            loopback_pin,
            twai::BaudRate::B1000K,
            TwaiMode::SelfTest,
        );

        config.set_filter(SingleStandardFilter::new(
            b"00000000000",
            b"x",
            [b"xxxxxxxx", b"xxxxxxxx"],
        ));

        let twai = config.start();

        Context { twai }
    }

    #[test]
    #[timeout(3)]
    fn test_send_receive(mut ctx: Context) {
        let frame = EspTwaiFrame::new_self_reception(StandardId::ZERO, &[1, 2, 3]).unwrap();
        block!(ctx.twai.transmit(&frame)).unwrap();

        let frame = block!(ctx.twai.receive()).unwrap();

        assert_eq!(frame.data(), &[1, 2, 3])
    }
}

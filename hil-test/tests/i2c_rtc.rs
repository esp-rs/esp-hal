//! RTC_I2C test

//% CHIPS: esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::time::Duration;

use esp_hal::i2c::rtc::{Config, I2c, Timing};

struct Context {
    i2c: I2c<'static>,
}

// HIL test device:
const DUT_ADDRESS: u8 = 0x77;
const READ_DATA_COMMAND: u8 = 0xAA;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (sda, scl) = hil_test::i2c_pins!(peripherals);

        let config = Config::default()
            .with_timing(Timing::standard_mode())
            .with_timeout(Duration::from_micros(100));
        let i2c = I2c::new(peripherals.RTC_I2C, config, sda, scl).unwrap();

        Context { i2c }
    }

    #[test]
    fn test_read_cali(mut ctx: Context) {
        let mut data = [0; 22];

        ctx.i2c
            .read(DUT_ADDRESS, READ_DATA_COMMAND, &mut data)
            .unwrap();

        assert_ne!(data, [0u8; 22]);
    }
}

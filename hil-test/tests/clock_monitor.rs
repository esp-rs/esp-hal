//! Clock Monitor Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{prelude::*, rtc_cntl::Rtc};
use hil_test as _;

struct Context<'a> {
    rtc: Rtc<'a>,
}

impl Context<'_> {
    pub fn init() -> Self {
        let (peripherals, clocks) = esp_hal::init(Config::default());
        let rtc = Rtc::new(peripherals.LPWR);

        Context { rtc }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        Context::init()
    }

    #[test]
    fn test_estimated_clock(mut ctx: Context<'static>) {
        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32c2")] {
                // 26 MHz
                let expected_range = 23..=29;
            } else if #[cfg(feature = "esp32h2")] {
                // 32 MHz
                let expected_range = 29..=35;
            } else {
                // 40 MHz
                let expected_range = 35..=45;
            }
        }

        let measured_frequency = ctx.rtc.estimate_xtal_frequency();
        defmt::assert!(
            expected_range.contains(&measured_frequency),
            "Measured frequency: {}",
            measured_frequency
        );
    }
}

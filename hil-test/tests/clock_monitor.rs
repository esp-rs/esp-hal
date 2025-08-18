//! Clock Monitor Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::rtc_cntl::Rtc;
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

struct Context<'a> {
    rtc: Rtc<'a>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let rtc = Rtc::new(peripherals.LPWR);

        Context { rtc }
    }

    #[test]
    fn test_estimated_clock(mut ctx: Context<'static>) {
        let expected_range = match () {
            _ if cfg!(esp32c2) => 23..=29,
            _ if cfg!(esp32h2) => 29..=35,
            _ => 35..=45,
        };

        let measured_frequency = ctx.rtc.estimate_xtal_frequency();
        hil_test::assert!(
            expected_range.contains(&measured_frequency),
            "Measured frequency: {}",
            measured_frequency
        );
    }
}

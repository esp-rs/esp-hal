//! PSRAM driver tests.

//% CHIP_FILTER: psram_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use esp_hal::psram::{Psram, PsramConfig};

    #[test]
    fn size_is_the_detected_size() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());

        let (_, mapped) = psram.raw_parts();
        assert!(mapped > 0);
        assert_eq!(psram.size(), mapped);
    }
}

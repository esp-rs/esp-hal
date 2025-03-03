//! PSRAM-related tests

//% CHIPS(quad): esp32 esp32s2
// The S3 dev kit in the HIL-tester has octal PSRAM.
//% CHIPS(octal): esp32s3
//% ENV(octal): ESP_HAL_CONFIG_PSRAM_MODE=octal
//% FEATURES: unstable psram

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 2)]
mod tests {
    #[init]
    fn init() {
        let p = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
    }

    #[test]
    fn test_simple() {
        let mut vec = alloc::vec::Vec::new();

        for i in 0..10000 {
            vec.push(i);
        }

        for i in 0..10000 {
            assert_eq!(vec[i], i);
        }
    }
}

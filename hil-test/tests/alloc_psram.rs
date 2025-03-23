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
<<<<<<< HEAD
#[embedded_test::tests]
=======
#[embedded_test::tests(default_timeout = 2)]
>>>>>>> d7af7dec (esp32s2: psram_cache_init typo fix (#3196))
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
<<<<<<< HEAD

    #[test]
    fn all_psram_is_usable() {
        let free = esp_alloc::HEAP.free();
        defmt::info!("Free: {}", free);
        let mut vec = alloc::vec::Vec::with_capacity(free);

        for i in 0..free {
            vec.push((i % 256) as u8);
        }

        for i in 0..free {
            assert_eq!(vec[i], (i % 256) as u8);
        }
    }
=======
>>>>>>> d7af7dec (esp32s2: psram_cache_init typo fix (#3196))
}

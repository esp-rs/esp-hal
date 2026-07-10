//! Allocator tests in DCache-reclaimed memory

//% CHIP_FILTER: esp32s3
//% ENV(esp32s3): ESP_HAL_CONFIG_DATA_CACHE_SIZE=32KB
//% FEATURES: esp-alloc

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

#[embedded_test::tests]
mod dcache_reclaimed {
    use esp_hal::{clock::CpuClock, ram};

    #[init]
    fn init() {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let _p = esp_hal::init(config);

        esp_alloc::heap_allocator!(#[ram(unstable(dcache_reclaimed))] size: 32 * 1024);
    }

    // alloc::vec::Vec tests

    #[test]
    fn test_simple() {
        let mut vec = alloc::vec::Vec::with_capacity(16384);

        for i in 0..16384 {
            vec.push((i % 256) as u8);
        }

        for i in 0..16384 {
            assert_eq!(vec[i], (i % 256) as u8);
        }
    }
}

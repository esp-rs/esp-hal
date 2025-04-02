//! Allocator and PSRAM-related tests

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
#[embedded_test::tests]
mod tests {
    use allocator_api2::vec::Vec;
    use esp_alloc::{AnyMemory, ExternalMemory, InternalMemory};

    #[init]
    fn init() {
        let p = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
    }

    // alloc::vec::Vec tests

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

    // allocator_api2 tests

    #[test]
    fn test_simple_with_allocator() {
        let mut vec = Vec::new_in(AnyMemory);

        for i in 0..10000 {
            vec.push(i);
        }

        for i in 0..10000 {
            assert_eq!(vec[i], i);
        }
    }

    #[test]
    #[should_panic]
    fn internal_allocator_can_not_allocate_from_psram() {
        let mut vec = Vec::new_in(InternalMemory);

        vec.push(0);
    }

    #[test]
    fn allocator_does_not_panic_if_fallible_allocation_fails() {
        let mut vec = Vec::<u8, _>::new_in(InternalMemory);
        assert!(vec.try_reserve(1_000_000_000).is_err());

        let mut vec = Vec::<u8, _>::new_in(ExternalMemory);
        assert!(vec.try_reserve(1_000_000_000).is_err());

        let mut vec = Vec::<u8, _>::new_in(AnyMemory);
        assert!(vec.try_reserve(1_000_000_000).is_err());
    }

    #[test]
    fn internal_allocator_can_allocate_memory() {
        esp_alloc::heap_allocator!(size: 64000);
        let mut vec: Vec<u32, _> = Vec::new_in(InternalMemory);

        vec.push(0xabcd1234);
        assert_eq!(vec[0], 0xabcd1234);
    }

    #[test]
    fn all_psram_is_usable_with_external_mem_allocator() {
        let free = esp_alloc::HEAP.free();
        defmt::info!("Free: {}", free);
        let mut vec = Vec::with_capacity_in(free, ExternalMemory);

        for i in 0..free {
            vec.push((i % 256) as u8);
        }

        for i in 0..free {
            assert_eq!(vec[i], (i % 256) as u8);
        }
    }

    #[test]
    fn all_psram_is_usable_with_any_mem_allocator() {
        let free = esp_alloc::HEAP.free();
        defmt::info!("Free: {}", free);
        let mut vec = Vec::with_capacity_in(free, AnyMemory);

        for i in 0..free {
            vec.push((i % 256) as u8);
        }

        for i in 0..free {
            assert_eq!(vec[i], (i % 256) as u8);
        }
    }
}

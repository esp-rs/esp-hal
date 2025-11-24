//! Allocator and PSRAM-related tests

//% CHIPS:
//% CHIPS(llff_quad, tlsf_quad): esp32 esp32s2
// The S3 dev kit in the HIL-tester has octal PSRAM.
//% CHIPS(llff_octal, tlsf_octal): esp32s3
//% ENV(llff_octal, tlsf_octal): ESP_HAL_CONFIG_PSRAM_MODE=octal
//% ENV(llff_octal, llff_quad): ESP_ALLOC_CONFIG_HEAP_ALGORITHM=LLFF
//% ENV(tlsf_octal, tlsf_quad): ESP_ALLOC_CONFIG_HEAP_ALGORITHM=TLSF
//% FEATURES: unstable psram esp-storage esp-alloc/nightly

#![no_std]
#![no_main]
// TODO: this test is Xtensa-only, so we can enable allocator_api unconditionally. This will not
// always be the case. Will this need a //% TOOLCHAIN?
#![feature(allocator_api)]

use hil_test as _;

extern crate alloc;

#[embedded_test::tests]
mod tests {
    use alloc::vec::Vec as AllocVec;

    use allocator_api2::vec::Vec;
    use embedded_storage::*;
    use esp_alloc::{AnyMemory, ExternalMemory, InternalMemory};
    use esp_bootloader_esp_idf::partitions;
    use esp_hal::peripherals::FLASH;
    use esp_storage::FlashStorage;

    struct Context<'a> {
        flash: FLASH<'a>,
    }

    #[init]
    fn init() -> Context<'static> {
        let p = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        Context { flash: p.FLASH }
    }

    // alloc::vec::Vec tests

    #[test]
    fn test_simple() {
        let mut vec = AllocVec::new();

        for i in 0..10000 {
            vec.push(i);
        }

        for i in 0..10000 {
            assert_eq!(vec[i], i);
        }
    }

    #[test]
    fn all_psram_is_usable() {
        if option_env!("ESP_ALLOC_CONFIG_HEAP_ALGORITHM") == Some("LLFF") {
            let free = esp_alloc::HEAP.free();
            defmt::info!("Free: {}", free);
            let mut vec = AllocVec::with_capacity(free);

            for i in 0..free {
                vec.push((i % 256) as u8);
            }

            for i in 0..free {
                assert_eq!(vec[i], (i % 256) as u8);
            }
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
        if option_env!("ESP_ALLOC_CONFIG_HEAP_ALGORITHM") == Some("LLFF") {
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
    }

    #[test]
    fn all_psram_is_usable_with_any_mem_allocator() {
        if option_env!("ESP_ALLOC_CONFIG_HEAP_ALGORITHM") == Some("LLFF") {
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

    #[test]
    fn test_with_accessing_flash_storage(ctx: Context<'static>) {
        let mut flash = FlashStorage::new(ctx.flash);

        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

        // The app descriptor (if present) is contained in the first 256 bytes
        // of an app image, right after the image header (24 bytes) and the first
        // section header (8 bytes)
        let mut app_desc = [0u8; 256];
        pt.find_partition(partitions::PartitionType::App(
            partitions::AppPartitionSubType::Factory,
        ))
        .unwrap()
        .unwrap()
        .as_embedded_storage(&mut flash)
        .read(32, &mut app_desc)
        .unwrap();

        let app_desc_wanted = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(hil_test::ESP_APP_DESC) as *const u8,
                256,
            )
        };

        assert_eq!(&app_desc_wanted, &app_desc);
    }

    #[test]
    fn test_spiram_is_reliable_when_using_esp_storage() {
        // adapted from the reproducer in https://github.com/esp-rs/esp-hal/issues/3642

        const NVS_PART_FLASH_ADDR: usize = 0x9000;

        #[repr(C, align(4))]
        struct AlignedBuf<const N: usize>([u8; N]);

        let rng = esp_hal::rng::Rng::new();

        for _ in 0..100 {
            let mut buf = [0u8; 1024];
            let mut heap_buf = alloc::vec![0u8; 1024];
            rng.read(&mut buf);
            heap_buf.copy_from_slice(&buf);
            let mut flash_buf = AlignedBuf([0; 4096]);
            unsafe {
                esp_storage::ll::spiflash_write(
                    NVS_PART_FLASH_ADDR as u32,
                    flash_buf.0.as_mut_ptr() as *const u32,
                    1024,
                )
                .unwrap();
            }

            for i in 0..1024 {
                assert_eq!(buf[i], heap_buf[i], "buf != heap_buf at index {}", i);
            }
        }
    }
}

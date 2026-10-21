//! Allocator and PSRAM-related tests

//% CHIP_FILTER(llff, tlsf): psram_driver_supported
//% ENV(llff): ESP_ALLOC_CONFIG_HEAP_ALGORITHM=LLFF
//% ENV(tlsf): ESP_ALLOC_CONFIG_HEAP_ALGORITHM=TLSF
// The default 64KB leaves nothing to reclaim, see the dcache_reclaimed tests below.
//% ENV-IF(esp32s3): ESP_HAL_CONFIG_DATA_CACHE_SIZE=32KB
//% FEATURES: unstable esp-alloc/nightly

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

#[embedded_test::tests(default_timeout = 30)] // P4 PSRAM can get large
mod tests {
    use alloc::vec::Vec as AllocVec;

    use allocator_api2::vec::Vec;
    use esp_alloc::{AnyMemory, ExternalMemory, InternalMemory};
    use esp_hal::clock::CpuClock;

    #[init]
    fn init() {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
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
}

/// Only the ESP32-S3 can hand unused data cache to the allocator, and only as much of it
/// as `ESP_HAL_CONFIG_DATA_CACHE_SIZE` leaves unused.
#[cfg(feature = "esp32s3")]
#[embedded_test::tests]
mod dcache_reclaimed {
    use esp_hal::{clock::CpuClock, ram};

    #[init]
    fn init() {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let _p = esp_hal::init(config);

        esp_alloc::heap_allocator!(#[ram(unstable(dcache_reclaimed))] size: 32 * 1024);
    }

    #[test]
    fn reclaimed_dcache_is_usable() {
        let mut vec = alloc::vec::Vec::with_capacity(16384);

        for i in 0..16384 {
            vec.push((i % 256) as u8);
        }

        for i in 0..16384 {
            assert_eq!(vec[i], (i % 256) as u8);
        }
    }
}

#[cfg(multi_core)]
#[embedded_test::tests]
mod multi_core_tests {
    use core::sync::atomic::{AtomicBool, Ordering};

    use allocator_api2::boxed::Box;
    use esp_alloc::ExternalMemory;
    use esp_hal::{
        clock::CpuClock,
        peripherals::CPU_CTRL,
        system::{CpuControl, Stack},
        time::{Duration, Instant},
    };
    use hil_test::mk_static;

    #[init]
    fn init() -> CPU_CTRL<'static> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        p.CPU_CTRL
    }

    #[test]
    fn test_other_core_writes_psram(cpu_ctrl: CPU_CTRL<'static>) {
        const PATTERN: u32 = 0xA5C3_5A3C;
        static WRITTEN: AtomicBool = AtomicBool::new(false);

        let word = Box::into_raw(Box::new_in(0u32, ExternalMemory)) as usize;

        let mut cpu_control = CpuControl::new(cpu_ctrl);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let _guard = cpu_control
            .start_app_core(stack, move || {
                unsafe { (word as *mut u32).write_volatile(PATTERN) };
                WRITTEN.store(true, Ordering::Release);
                loop {}
            })
            .unwrap();

        let deadline = Instant::now() + Duration::from_millis(100);
        while !WRITTEN.load(Ordering::Acquire) {
            assert!(
                Instant::now() < deadline,
                "the other core did not finish its write"
            );
        }
        assert_eq!(unsafe { (word as *const u32).read_volatile() }, PATTERN);
    }
}

#[cfg(flash_driver_supported)]
#[embedded_test::tests]
mod flash_tests {
    use esp_bootloader_esp_idf::partitions;
    use esp_hal::{
        clock::CpuClock,
        flash::{Config, Flash},
        peripherals::FLASH,
    };

    struct Context<'a> {
        flash: FLASH<'a>,
    }

    #[init]
    fn init() -> Context<'static> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        Context { flash: p.FLASH }
    }

    #[test]
    fn test_with_accessing_flash(ctx: Context<'static>) {
        let mut flash = Flash::new(ctx.flash, Config::default()).unwrap();

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
        .as_auto_flash_region(&mut flash)
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
    fn test_spiram_is_reliable_when_writing_flash(ctx: Context<'static>) {
        // adapted from the reproducer in https://github.com/esp-rs/esp-hal/issues/3642

        const NVS_PART_FLASH_ADDR: u32 = 0x9000;

        let mut flash = Flash::new(ctx.flash, Config::default()).unwrap();
        let rng = esp_hal::rng::Rng::new();

        for _ in 0..100 {
            let mut buf = [0u8; 1024];
            let mut heap_buf = alloc::vec![0u8; 1024];
            rng.read(&mut buf);
            heap_buf.copy_from_slice(&buf);
            let flash_buf = [0u32; 256];
            // SAFETY: the NVS partition is not mapped.
            unsafe { flash.write(NVS_PART_FLASH_ADDR, &flash_buf) }.unwrap();

            for i in 0..1024 {
                assert_eq!(buf[i], heap_buf[i], "buf != heap_buf at index {}", i);
            }
        }
    }
}

/// PSRAM at the default 200 MHz with the MPLL at 400 MHz, and at 250 MHz with the MPLL at
/// 500 MHz.
#[cfg(feature = "esp32s31")]
#[embedded_test::tests]
mod mpll {
    use esp_hal::{
        clock::{
            ClockConfig,
            CpuClock,
            ll::{MpllClkConfig, mpll_clk_frequency},
        },
        peripherals::{HP_SYS_CLKRST, LP_AON_CLK_RST},
        psram::{Psram, PsramConfig, PsramTimingParams},
    };

    fn assert_mpll_programmed(expected_hz: u32) {
        let div = unsafe { LP_AON_CLK_RST::steal() }
            .register_block()
            .mspi_div()
            .read();
        let fb_div = div.mspi_fb_div().bits() as u32;
        let ref_div = div.mspi_ref_div().bits() as u32;
        let mpll_hz = 40_000_000 * (fb_div + 1) / (ref_div + 1);

        hil_test::assert_eq!(mpll_hz, expected_hz);
        hil_test::assert_eq!(mpll_hz, mpll_clk_frequency());

        let cal = unsafe { HP_SYS_CLKRST::steal() }
            .register_block()
            .ana_pll_ctrl0()
            .read();
        hil_test::assert!(
            cal.mspi_cal_stop().bit_is_set(),
            "esp-hal's MPLL calibration did not run"
        );
    }

    fn assert_psram_holds_a_pattern(psram: &Psram) {
        let (start, size) = psram.raw_parts();
        hil_test::assert!(size > 0);

        let words = size / 4;
        let psram = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, words) };

        for (i, word) in psram.iter_mut().enumerate() {
            *word = (i as u32).wrapping_mul(0x9E37_79B9);
        }
        for (i, word) in psram.iter().enumerate() {
            hil_test::assert_eq!(*word, (i as u32).wrapping_mul(0x9E37_79B9));
        }
    }

    #[test]
    fn default_mpll_400_psram_200() {
        let p = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(ClockConfig::from(CpuClock::max())),
        );
        let psram = Psram::new(p.PSRAM, PsramConfig::default());

        assert_mpll_programmed(400_000_000);
        assert_psram_holds_a_pattern(&psram);
    }

    #[test]
    fn mpll_500_psram_250() {
        let mut clocks = ClockConfig::from(CpuClock::max());
        clocks.mpll_clk = Some(MpllClkConfig::_500);
        let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clocks));
        let config = PsramConfig {
            timing: PsramTimingParams::MHZ_250,
            ..Default::default()
        };
        let psram = Psram::new(p.PSRAM, config);

        assert_mpll_programmed(500_000_000);
        assert_psram_holds_a_pattern(&psram);
    }
}

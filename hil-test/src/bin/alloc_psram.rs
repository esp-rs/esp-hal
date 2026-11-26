//! Allocator and PSRAM-related tests

//% CHIP_FILTER(llff, tlsf): psram_driver_supported
//% ENV(llff): ESP_ALLOC_CONFIG_HEAP_ALGORITHM=LLFF
//% ENV(tlsf): ESP_ALLOC_CONFIG_HEAP_ALGORITHM=TLSF
// Just below 512 KiB, so that only the second-level rounding of TLSF needs the next size class.
//% ENV(tlsf): ESP_ALLOC_CONFIG_TLSF_MAX_ALLOCATION=524256
// The default 64KB leaves nothing to reclaim, see the dcache_reclaimed tests below.
//% ENV-IF(esp32s3): ESP_HAL_CONFIG_DATA_CACHE_SIZE=32KB
//% FEATURES: unstable esp-alloc/nightly

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

/// A byte pattern that does not repeat at a power-of-two offset, so that an address wrap fails
/// the read-back.
fn pattern(i: usize) -> u8 {
    ((i as u32).wrapping_mul(0x9E37_79B9) >> 24) as u8
}

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
        hil_test::init_logger!(p);
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
                vec.push(crate::pattern(i));
            }

            for i in 0..free {
                assert_eq!(vec[i], crate::pattern(i));
            }
        }
    }

    #[test]
    fn tlsf_max_allocation_is_applied() {
        if option_env!("ESP_ALLOC_CONFIG_HEAP_ALGORITHM") == Some("TLSF") {
            // The configured maximum needs `FLLEN = 16`. Its largest size class starts at
            // `2^19 + 31 * 2^14` bytes, which must also hold the 8 byte block header.
            const TOO_LARGE: usize = (1 << 19) + 31 * (1 << 14) - 8 + 1;

            // The heap splits a region into pools of `GRANULARITY << FLLEN` (1 MiB) bytes. With
            // this much free memory, one pool is complete, and its free block can hold `TOO_LARGE`.
            assert!(esp_alloc::HEAP.free() >= 1024 * 1024);

            let mut vec = AllocVec::<u8>::new();
            assert!(vec.try_reserve_exact(TOO_LARGE).is_err());
            assert!(vec.try_reserve_exact(524256).is_ok());
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
                vec.push(crate::pattern(i));
            }

            for i in 0..free {
                assert_eq!(vec[i], crate::pattern(i));
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
                vec.push(crate::pattern(i));
            }

            for i in 0..free {
                assert_eq!(vec[i], crate::pattern(i));
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
        let peripherals = esp_hal::init(config);
        hil_test::init_logger!(peripherals);

        esp_alloc::heap_allocator!(#[ram(unstable(dcache_reclaimed))] size: 32 * 1024);
    }

    #[test]
    fn reclaimed_dcache_is_usable() {
        let mut vec = alloc::vec::Vec::with_capacity(16384);

        for i in 0..16384 {
            vec.push(crate::pattern(i));
        }

        for i in 0..16384 {
            assert_eq!(vec[i], crate::pattern(i));
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
        hil_test::init_logger!(p);
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

#[cfg(all(multi_core, dma_can_access_psram))]
#[embedded_test::tests(default_timeout = 30)]
mod multi_core_cache_tests {
    use core::{
        ptr,
        sync::atomic::{AtomicBool, AtomicU32, Ordering},
    };

    use allocator_api2::{boxed::Box, vec::Vec};
    use esp_alloc::ExternalMemory;
    use esp_hal::{
        clock::CpuClock,
        dma::aligned::DmaAlignedMut,
        peripherals::CPU_CTRL,
        system::{CpuControl, Stack},
        time::{Duration, Instant},
    };
    use hil_test::mk_static;

    /// The largest PSRAM cache line size of the supported chips.
    #[derive(Clone, Copy)]
    #[repr(C, align(128))]
    struct PsramLine([u32; 32]);

    #[init]
    fn init() -> CPU_CTRL<'static> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);
        hil_test::init_logger!(p);
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        p.CPU_CTRL
    }

    /// The other core writes back a large PSRAM buffer in a loop. This core writes `line`, then
    /// writes back, invalidates and reads `line`. A lost write-back makes the read return old data.
    fn writeback_is_not_lost_during_other_core_writeback(
        cpu_ctrl: CPU_CTRL<'static>,
        mut line: DmaAlignedMut<'static, [u32]>,
    ) {
        // Stays below the 524256 byte `ESP_ALLOC_CONFIG_TLSF_MAX_ALLOCATION` of the `tlsf`
        // configuration. A 1 MiB buffer cannot be allocated there.
        const BIG_LINES: usize = 384 * 1024 / size_of::<PsramLine>();
        static STOP: AtomicBool = AtomicBool::new(false);
        static STOPPED: AtomicBool = AtomicBool::new(false);
        static ROUNDS: AtomicU32 = AtomicU32::new(0);

        let mut big = Vec::with_capacity_in(BIG_LINES, ExternalMemory);
        big.resize(BIG_LINES, PsramLine([0; 32]));
        let mut big = DmaAlignedMut::new(Box::leak(big.into_boxed_slice())).unwrap();

        let mut cpu_control = CpuControl::new(cpu_ctrl);
        let stack = mk_static!(Stack<8192>, Stack::new());
        // Dropping the guard parks the other core. Stop the loop first, so that the other core
        // does not stop in a cache operation.
        let _guard = cpu_control
            .start_app_core(stack, move || {
                let mut round = 0u32;
                while !STOP.load(Ordering::Acquire) {
                    round = round.wrapping_add(1);
                    for big_line in big.iter_mut() {
                        unsafe { ptr::write_volatile(&mut big_line.0[0], round) };
                    }
                    big.writeback();
                    ROUNDS.store(round, Ordering::Release);
                }
                STOPPED.store(true, Ordering::Release);
                loop {}
            })
            .unwrap();

        while ROUNDS.load(Ordering::Acquire) == 0 {}

        let mut value = 0u32;
        let mut lost = 0u32;
        let end = Instant::now() + Duration::from_secs(5);
        while Instant::now() < end {
            value = value.wrapping_add(1);
            for word in line.iter_mut() {
                unsafe { ptr::write_volatile(word, value) };
            }
            line.writeback();
            line.invalidate();
            if line
                .iter()
                .any(|word| unsafe { ptr::read_volatile(word) } != value)
            {
                lost += 1;
            }
        }

        STOP.store(true, Ordering::Release);
        while !STOPPED.load(Ordering::Acquire) {}

        defmt::info!(
            "{} of {} write-backs lost, {} rounds on the other core",
            lost,
            value,
            ROUNDS.load(Ordering::Relaxed)
        );
        hil_test::assert_eq!(lost, 0);
    }

    #[test]
    fn psram_writeback_is_not_lost_during_other_core_writeback(cpu_ctrl: CPU_CTRL<'static>) {
        let line = Box::leak(Box::new_in(PsramLine([0; 32]), ExternalMemory));
        let line = DmaAlignedMut::new(&mut line.0).unwrap().unsize();

        writeback_is_not_lost_during_other_core_writeback(cpu_ctrl, line);
    }

    #[test]
    #[cfg(soc_internal_memory_cached)]
    fn internal_writeback_is_not_lost_during_other_core_writeback(cpu_ctrl: CPU_CTRL<'static>) {
        use esp_hal::dma::aligned::InternalMemory;

        let line = mk_static!(InternalMemory<[u32; 16]>, InternalMemory::new([0; 16]));

        writeback_is_not_lost_during_other_core_writeback(cpu_ctrl, line.get_mut().unsize());
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
        hil_test::init_logger!(p);
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

/// In `PsramVaddrMode::Normal`, each core caches PSRAM in its own cache.
#[cfg(feature = "esp32")]
#[embedded_test::tests(default_timeout = 10)]
mod normal_vaddr_mode_tests {
    use core::sync::atomic::{AtomicU8, AtomicUsize, Ordering};

    use esp_hal::{
        clock::CpuClock,
        flash::{Config, Flash},
        peripherals::{CPU_CTRL, FLASH},
        psram::{Psram, PsramConfig, PsramVaddrMode},
        system::{CpuControl, Stack},
    };
    use hil_test::mk_static;

    struct Context {
        cpu_ctrl: CPU_CTRL<'static>,
        flash: FLASH<'static>,
        psram: Psram,
    }

    #[init]
    fn init() -> Context {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);
        hil_test::init_logger!(p);
        let psram = Psram::new(
            p.PSRAM,
            PsramConfig {
                psram_vaddr_mode: PsramVaddrMode::Normal,
                ..Default::default()
            },
        );

        Context {
            cpu_ctrl: p.CPU_CTRL,
            flash: p.FLASH,
            psram,
        }
    }

    #[test]
    fn flash_write_keeps_psram_writes_of_the_other_core(ctx: Context) {
        const NVS_PART_FLASH_ADDR: u32 = 0x9000;
        const LEN: usize = 1024;
        const WRITTEN: u8 = 1;
        const FLASHED: u8 = 2;
        const CHECKED: u8 = 3;
        static STATE: AtomicU8 = AtomicU8::new(0);
        static LOST: AtomicUsize = AtomicUsize::new(0);

        let value = |i: usize| (i as u8) ^ 0x5A;

        // This core does not access the buffer, so only the cache of the other core holds it.
        let (start, size) = ctx.psram.raw_parts();
        assert!(size >= LEN);
        let buf = start as usize;

        let mut cpu_control = CpuControl::new(ctx.cpu_ctrl);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let _guard = cpu_control
            .start_app_core(stack, move || {
                let buf = buf as *mut u8;
                for i in 0..LEN {
                    unsafe { buf.add(i).write_volatile(value(i)) };
                }
                STATE.store(WRITTEN, Ordering::Release);
                while STATE.load(Ordering::Acquire) != FLASHED {}
                let lost = (0..LEN)
                    .filter(|&i| unsafe { buf.add(i).read_volatile() } != value(i))
                    .count();
                LOST.store(lost, Ordering::Relaxed);
                STATE.store(CHECKED, Ordering::Release);
                loop {}
            })
            .unwrap();

        while STATE.load(Ordering::Acquire) != WRITTEN {}
        let mut flash = Flash::new(ctx.flash, Config::default()).unwrap();
        let flash_buf = [0u32; 256];
        // SAFETY: the NVS partition is not mapped.
        unsafe { flash.write(NVS_PART_FLASH_ADDR, &flash_buf) }.unwrap();
        STATE.store(FLASHED, Ordering::Release);

        while STATE.load(Ordering::Acquire) != CHECKED {}
        assert_eq!(LOST.load(Ordering::Relaxed), 0);
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
            ll::{ClockTree, MpllClkConfig, configure_mpll_clk, mpll_clk_frequency},
        },
        peripherals::{HP_SYS_CLKRST, LP_AON_CLK_RST, MEMSPI2},
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

    /// Asserts that PSRAM runs from the MPLL, at half the MPLL rate.
    fn assert_psram_on_half_mpll() {
        let source = HP_SYS_CLKRST::regs()
            .psram_ctrl0()
            .read()
            .clk_src_sel()
            .bits();
        hil_test::assert_eq!(source, 1, "PSRAM does not run from the MPLL");
        let bits = MEMSPI2::regs().sram_clk().read().bits();
        hil_test::assert_eq!((bits & 0xff) + 1, 2, "the PSRAM bus clock divider is not 2");
    }

    /// The word at `i` of the pattern for `seed`.
    fn pattern_word(seed: u32, i: usize) -> u32 {
        (i as u32 ^ seed).wrapping_mul(0x9E37_79B9) ^ seed
    }

    fn psram_words(psram: &Psram) -> &'static mut [u32] {
        let (start, size) = psram.raw_parts();
        hil_test::assert!(size > 0);
        unsafe { core::slice::from_raw_parts_mut(start as *mut u32, size / 4) }
    }

    fn fill_pattern(psram: &Psram) {
        for (i, word) in psram_words(psram).iter_mut().enumerate() {
            *word = pattern_word(0, i);
        }
    }

    fn assert_pattern(psram: &Psram) {
        let first_difference = psram_words(psram)
            .iter()
            .enumerate()
            .position(|(i, &word)| word != pattern_word(0, i));
        hil_test::assert_eq!(first_difference, None);
    }

    #[test]
    fn default_mpll_400_psram_200() {
        let p = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(ClockConfig::from(CpuClock::max())),
        );
        hil_test::init_logger!(p);
        let psram = Psram::new(p.PSRAM, PsramConfig::default());

        assert_mpll_programmed(400_000_000);
        fill_pattern(&psram);
        assert_pattern(&psram);
    }

    // Fails if the hardware keeps the old MPLL rate, or if PSRAM loses its content in the change.
    #[test]
    fn mpll_change_after_psram_new() {
        let p = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(ClockConfig::from(CpuClock::max())),
        );
        hil_test::init_logger!(p);
        let psram = Psram::new(p.PSRAM, PsramConfig::default());
        fill_pattern(&psram);

        for (mpll, hz) in [
            (MpllClkConfig::_500, 500_000_000),
            (MpllClkConfig::_400, 400_000_000),
        ] {
            ClockTree::with(|clocks| configure_mpll_clk(clocks, mpll));
            assert_mpll_programmed(hz);
            assert_pattern(&psram);
        }
    }

    #[test]
    fn mpll_500_psram_250() {
        let mut clocks = ClockConfig::from(CpuClock::max());
        clocks.mpll_clk = Some(MpllClkConfig::_500);
        let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clocks));
        hil_test::init_logger!(p);
        let config = PsramConfig {
            timing: PsramTimingParams::MHZ_250,
            ..Default::default()
        };
        let psram = Psram::new(p.PSRAM, config);

        assert_mpll_programmed(500_000_000);
        fill_pattern(&psram);
        assert_pattern(&psram);
    }

    // The APP core reads PSRAM in a loop while the MPLL moves from 400 to 500 MHz and back. Both
    // patterns are larger than the data cache. Fails if the APP core does not continue after a
    // change or reads wrong data, or if this core's pattern changes.
    #[test]
    #[timeout(10)]
    fn mpll_change_with_app_core_running() {
        use core::sync::atomic::{AtomicU32, Ordering};

        use esp_hal::{
            system::{CpuControl, Stack},
            time::{Duration, Instant},
        };

        /// 256 KiB of `u32` words, more than the data cache holds.
        const UNCACHED_WORDS: usize = 0x1_0000;

        static READS: AtomicU32 = AtomicU32::new(0);
        static WRONG: AtomicU32 = AtomicU32::new(0);

        let peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(peripherals);
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let words = psram_words(&psram);
        hil_test::assert!(words.len() >= 2 * UNCACHED_WORDS);
        let (local, shared) = words[..2 * UNCACHED_WORDS].split_at_mut(UNCACHED_WORDS);
        // PSRAM keeps its content across the reset between two tests, so the seed differs from
        // run to run.
        let seed = esp_hal::rng::Rng::new().random();
        for (i, word) in local.iter_mut().enumerate() {
            *word = pattern_word(seed, i);
        }
        let shared_seed = !seed;
        for (i, word) in shared.iter_mut().enumerate() {
            *word = pattern_word(shared_seed, i);
        }
        let shared = shared.as_ptr() as usize;

        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
        let stack = hil_test::mk_static!(Stack<8192>, Stack::new());
        let _guard = cpu_control
            .start_app_core(stack, move || {
                loop {
                    for i in 0..UNCACHED_WORDS {
                        let word = unsafe { (shared as *const u32).add(i).read_volatile() };
                        if word != pattern_word(shared_seed, i) {
                            WRONG.fetch_add(1, Ordering::Relaxed);
                        }
                        READS.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })
            .unwrap();

        // Waits until the APP core reads the whole pattern once more.
        let assert_app_core_reads = || {
            let before = READS.load(Ordering::Relaxed);
            let deadline = Instant::now() + Duration::from_millis(500);
            while READS.load(Ordering::Relaxed).wrapping_sub(before) < UNCACHED_WORDS as u32 {
                hil_test::assert!(Instant::now() < deadline, "the APP core stopped");
            }
            hil_test::assert_eq!(
                WRONG.load(Ordering::Relaxed),
                0,
                "the APP core read wrong data"
            );
        };

        assert_app_core_reads();
        for (mpll, hz) in [
            (MpllClkConfig::_500, 500_000_000),
            (MpllClkConfig::_400, 400_000_000),
        ] {
            ClockTree::with(|clocks| configure_mpll_clk(clocks, mpll));
            hil_test::assert_eq!(ClockTree::with(|clocks| clocks.mpll_clk()), Some(mpll));
            assert_mpll_programmed(hz);
            assert_psram_on_half_mpll();
            assert_app_core_reads();
            let first_difference = local
                .iter()
                .enumerate()
                .position(|(i, &word)| word != pattern_word(seed, i));
            hil_test::assert_eq!(first_difference, None, "this core's pattern changed");
        }
    }
}

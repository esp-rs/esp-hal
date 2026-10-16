//! Code in PSRAM: the firmware's code and read-only data run from the PSRAM window.

//% CHIP_FILTER(at_200mhz, at_250mhz): psram_supports_code_in_psram
//% ENV: ESP_HAL_CONFIG_CODE_IN_PSRAM=true
//% ENV(at_250mhz): ESP_HAL_CONFIG_PSRAM_FREQUENCY=250MHz
//% FEATURES: unstable esp-alloc embassy

#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use esp_hal::interrupt::software::SoftwareInterrupt;
use hil_test as _;

#[macro_use]
extern crate esp_metadata_generated;

static RODATA: [u32; 4] = [0x5053_524D; 4];
static mut DATA: u32 = 0x4441_5441;
static mut BSS: u32 = 0;

/// A random 64 KiB block after the image in the booted partition, so that each run wears another.
fn scratch_block(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>) -> u32 {
    use esp_bootloader_esp_idf::partitions;

    const BLOCK: u32 = 0x1_0000;
    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();
    let app = pt.booted_partition().unwrap().unwrap();

    // ESP-IDF image: a 24-byte header, then segments of load address, length, and data, then a
    // checksum and a hash, which fit in the next block.
    let mut header = [0u32; 6];
    flash.read(app.offset(), &mut header).unwrap();
    let mut end = app.offset() + 24;
    for _ in 0..(header[0] >> 8) & 0xFF {
        let mut segment = [0u32; 2];
        flash.read(end, &mut segment).unwrap();
        end += 8 + segment[1];
    }
    let first = (end + BLOCK).div_ceil(BLOCK);
    let last = (app.offset() + app.len()) / BLOCK;
    assert!(
        first < last,
        "no free 64 KiB block after the image in the booted partition"
    );
    (first + esp_hal::rng::Rng::new().random() % (last - first)) * BLOCK
}

/// 16 KiB of pattern for [`write_block`]. A `static mut` with a value other than zero is in
/// internal RAM, where a ROM flash routine can read it, and it is never written.
static mut BLOCK_PATTERN: [u32; 4096] = [0x5AA5_C33C; 4096];

/// Writes the pattern over the 64 KiB block at `block`.
///
/// A written block takes longer to erase than a blank block, which gives a test more chances to
/// observe the erase. No test depends on the length.
fn write_block(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>, block: u32) {
    // SAFETY: nothing writes `BLOCK_PATTERN`.
    let pattern = unsafe { &*core::ptr::addr_of!(BLOCK_PATTERN) };
    for offset in (0..0x1_0000).step_by(core::mem::size_of_val(pattern)) {
        // SAFETY: the callers pass a block in the booted partition, after the image.
        unsafe { flash.write(block + offset, pattern) }.unwrap();
    }
}

/// The number of `u32` words in 256 KiB, the size of the PSRAM data that the tests write and read
/// back. It is four times the largest data cache of these chips, 64 KiB on the ESP32-S31, so most
/// of it reaches PSRAM, and most of the read-back comes from PSRAM, not from the cache.
const UNCACHED_WORDS: usize = 0x1_0000;

/// The word at `i` of the pattern for `seed`. It differs from word to word and from seed to seed.
fn pattern_word(seed: u32, i: usize) -> u32 {
    (i as u32 ^ seed).wrapping_mul(0x9E37_79B9) ^ seed
}

/// A seed for [`fill_pattern`] that differs from run to run. PSRAM keeps its content across the
/// reset between two tests, so a fixed seed could find the last run's pattern still there.
fn fresh_seed() -> u32 {
    esp_hal::rng::Rng::new().random()
}

fn fill_pattern(words: &mut [u32], seed: u32) {
    for (i, word) in words.iter_mut().enumerate() {
        *word = pattern_word(seed, i);
    }
}

/// Asserts that `words` hold the pattern for `seed`, and names the first word that does not.
fn assert_pattern(words: &[u32], seed: u32, what: &str) {
    let first_difference = words
        .iter()
        .enumerate()
        .position(|(i, &word)| word != pattern_word(seed, i));
    assert_eq!(first_difference, None, "{what}: the pattern differs");
}

static SWINT: Mutex<RefCell<Option<SoftwareInterrupt<'static, 2>>>> =
    Mutex::new(RefCell::new(None));
static INTERRUPTS: AtomicU32 = AtomicU32::new(0);

#[inline(never)]
fn in_text() -> u32 {
    unsafe { core::ptr::read_volatile(&RODATA[0]) }
}

extern "C" fn swint_handler() {
    INTERRUPTS.fetch_add(1, Ordering::Relaxed);
    critical_section::with(|cs| {
        if let Some(swint) = SWINT.borrow_ref(cs).as_ref() {
            swint.reset();
        }
    });
}

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use core::ptr::{addr_of, addr_of_mut};

    use esp_bootloader_esp_idf::partitions::{self, AppPartitionSubType, PartitionType};
    #[cfg(feature = "esp32s31")]
    use esp_hal::{
        clock::{
            ClockConfig,
            ll::{ClockTree, mpll_clk_frequency, request_mpll_clk},
        },
        psram::PsramTimingParams,
    };
    use esp_hal::{
        flash::{Config, Flash},
        interrupt::{InterruptHandler, Priority},
        psram::{Psram, PsramConfig},
    };

    use super::*;

    const WINDOW: u32 = property!("psram.extmem_origin");
    const WINDOW_SIZE: u32 = property!("mmu.entry_num") * property!("mmu.page_size");
    const PATTERN: u32 = 0xA5C3_5A3C;

    fn in_window(addr: u32) -> bool {
        (WINDOW..WINDOW + WINDOW_SIZE).contains(&addr)
    }

    fn booted_app_offset(flash: &mut Flash<'_, esp_hal::Blocking>) -> u32 {
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();
        pt.booted_partition().unwrap().unwrap().offset()
    }

    #[test]
    fn test_code_in_psram() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(in_text as *const () as u32));
        assert_eq!(in_text(), 0x5053_524D);
    }

    #[test]
    fn test_rodata_in_psram() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(addr_of!(RODATA) as u32));
        // A volatile read, so the compiler cannot answer from the initializer.
        assert_eq!(
            unsafe { addr_of!(RODATA).read_volatile() },
            [0x5053_524D; 4]
        );
    }

    #[test]
    fn test_data_and_bss_are_initialized() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert_eq!(unsafe { addr_of!(DATA).read_volatile() }, 0x4441_5441);
        assert_eq!(unsafe { addr_of!(BSS).read_volatile() }, 0);
        unsafe { addr_of_mut!(BSS).write_volatile(PATTERN) };
        assert_eq!(unsafe { addr_of!(BSS).read_volatile() }, PATTERN);
    }

    // Reads go through the data cache, so this test shows the content and bounds of the copy, not
    // the write-back.
    #[test]
    #[timeout(30)]
    fn test_segments_match_image() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        let app = booted_app_offset(&mut flash);

        // ESP-IDF image: a 24-byte header, then segments of load address, length, and data.
        let mut header = [0u32; 6];
        flash.read(app, &mut header).unwrap();
        assert_eq!(header[0] & 0xFF, 0xE9);
        let segments = (header[0] >> 8) & 0xFF;

        let mut offset = app + 24;
        let mut compared = 0;
        for _ in 0..segments {
            let mut segment = [0u32; 2];
            flash.read(offset, &mut segment).unwrap();
            let [load, len] = segment;
            offset += 8;

            if in_window(load) {
                let mut chunk = [0u32; 256];
                let mut done = 0;
                while done < len {
                    let words = ((len - done) / 4).min(chunk.len() as u32) as usize;
                    flash.read(offset + done, &mut chunk[..words]).unwrap();
                    let mapped =
                        unsafe { core::slice::from_raw_parts((load + done) as *const u32, words) };
                    let first_difference =
                        chunk[..words].iter().zip(mapped).position(|(a, b)| a != b);
                    assert_eq!(first_difference, None, "in the 1 KiB at {:#x}", load + done);
                    done += words as u32 * 4;
                }
                compared += len;
            }
            offset += len;
        }
        assert!(compared > 0);
    }

    #[test]
    fn test_heap_after_code() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());

        let (start, size) = psram.raw_parts();
        assert!(start as u32 > in_text as *const () as u32);
        assert!(start as u32 > addr_of!(RODATA) as u32);

        let words = size / 4;
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, words) };
        let probes = [0, words / 2, words - 1];
        for (n, &i) in probes.iter().enumerate() {
            heap[i] = PATTERN.wrapping_add(n as u32);
        }
        for (n, &i) in probes.iter().enumerate() {
            assert_eq!(heap[i], PATTERN.wrapping_add(n as u32));
        }
    }

    #[test]
    fn test_heap_survives_flash_read() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        let app = booted_app_offset(&mut flash);

        let (start, _) = psram.raw_parts();
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, UNCACHED_WORDS) };
        let seed = fresh_seed();
        fill_pattern(heap, seed);

        let mut buffer = [0u32; 4096];
        flash.read(app, &mut buffer).unwrap();

        assert_pattern(heap, seed, "the heap after a flash read");
    }

    #[test]
    fn test_interrupt_handler_in_psram() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mut swint = SoftwareInterrupt::new(peripherals.FROM_CPU_INTR2);
        critical_section::with(|cs| {
            swint.set_interrupt_handler(InterruptHandler::new(swint_handler, Priority::Priority1));
            SWINT.borrow_ref_mut(cs).replace(swint);
        });
        critical_section::with(|cs| SWINT.borrow_ref(cs).as_ref().unwrap().raise());

        while INTERRUPTS.load(Ordering::Relaxed) == 0 {}
        assert!(in_window(swint_handler as *const () as u32));
    }

    /// The MPLL rate that the hardware is set to, from the ESP32-S31's MPLL feedback divider, with
    /// the 40 MHz crystal and the reference divider of 2 that the start-up uses.
    #[cfg(feature = "esp32s31")]
    fn mpll_in_hardware() -> u32 {
        let fb_div = esp_hal::peripherals::LP_AON_CLK_RST::regs()
            .mspi_div()
            .read()
            .mspi_fb_div()
            .bits();
        (u32::from(fb_div) + 1) * 40_000_000 / 2
    }

    // The ESP32-S31 clocks PSRAM from the MPLL, at 400 MHz for 200 MHz PSRAM and at 500 MHz for
    // 250 MHz PSRAM. The clock tree's rate, which drivers use, and the hardware's must both match.
    #[cfg(feature = "esp32s31")]
    #[test]
    fn test_mpll_rate() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        let expected = match option_env!("ESP_HAL_CONFIG_PSRAM_FREQUENCY") {
            Some("250MHz") => 500_000_000,
            _ => 400_000_000,
        };
        assert_eq!(mpll_clk_frequency(), expected);
        assert_eq!(mpll_in_hardware(), expected);
    }

    #[cfg(psram_separate_window)]
    #[test]
    fn test_no_flash_page_mapped() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        let spi0 = esp_hal::peripherals::SPI0::regs();

        for i in 0..1024 {
            spi0.mmu_item_index().write(|w| unsafe { w.bits(i) });
            assert!(
                !spi0.mmu_item_content().read().valid().bit_is_set(),
                "entry {} is mapped",
                i
            );
        }
    }

    // The start-up's MPLL rate stays when the configuration leaves the MPLL out.
    #[cfg(feature = "esp32s31")]
    #[test]
    fn test_mpll_rate_without_mpll_config() {
        let mut clocks = ClockConfig::from(esp_hal::clock::CpuClock::default());
        clocks.mpll_clk = None;
        let _peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clocks));

        let expected = match option_env!("ESP_HAL_CONFIG_PSRAM_FREQUENCY") {
            Some("250MHz") => 500_000_000,
            _ => 400_000_000,
        };
        assert_eq!(mpll_clk_frequency(), expected);
        assert_eq!(mpll_in_hardware(), expected);
    }

    #[cfg(feature = "esp32s31")]
    #[test]
    fn test_mpll_request_after_init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let (start, _) = psram.raw_parts();
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, UNCACHED_WORDS) };
        let seed = fresh_seed();
        fill_pattern(heap, seed);
        let before = mpll_clk_frequency();

        ClockTree::with(request_mpll_clk);

        assert_eq!(mpll_clk_frequency(), before);
        assert_eq!(mpll_in_hardware(), before);
        assert_eq!(in_text(), 0x5053_524D);
        assert_pattern(heap, seed, "the heap after the MPLL request");
    }

    #[cfg(soc_has_clock_node_mpll_clk)]
    #[test]
    fn test_psram_new_with_another_frequency() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let before = mpll_clk_frequency();
        let timing = match option_env!("ESP_HAL_CONFIG_PSRAM_FREQUENCY") {
            Some("250MHz") => PsramTimingParams::MHZ_200,
            _ => PsramTimingParams::MHZ_250,
        };

        let psram = Psram::new(
            peripherals.PSRAM,
            PsramConfig {
                timing,
                ..PsramConfig::default()
            },
        );

        let (start, size) = psram.raw_parts();
        assert!(size >= UNCACHED_WORDS * 4);
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, UNCACHED_WORDS) };
        let seed = fresh_seed();
        fill_pattern(heap, seed);
        assert_pattern(heap, seed, "the heap after Psram::new");
        assert_eq!(mpll_clk_frequency(), before);
        assert_eq!(mpll_in_hardware(), before);
        assert_eq!(in_text(), 0x5053_524D);
    }

    #[test]
    fn test_booted_partition() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();

        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(&mut flash, &mut pt_mem).unwrap();

        let booted = pt.booted_partition().unwrap().unwrap();
        assert_eq!(
            booted.partition_type(),
            PartitionType::App(AppPartitionSubType::Factory)
        );
    }
}

/// Flash operations while code runs from PSRAM: interrupts keep running, and on a chip with two
/// cores the other core too.
///
/// A periodic timer interrupt on the core that runs the operation is the probe, and core 1, where
/// there is one, counts its own ticks. Each operation here, an erase, a write, or a read, is one
/// ROM call, and the tests count the probe's runs and core 1's ticks during that call. A count well
/// above what a call that stopped them allows shows that they ran during the call. The probe's
/// handler runs from PSRAM, so on the ESP32-C5, whose flash and PSRAM share the MSPI bus, a run
/// during the call can also fetch from PSRAM while the ROM drives the flash.
#[embedded_test::tests(default_timeout = 30)]
mod flash_operations {
    use core::sync::atomic::AtomicBool;

    use allocator_api2::{boxed::Box, vec::Vec};
    use esp_alloc::ExternalMemory;
    use esp_bootloader_esp_idf::partitions;
    use esp_hal::{
        Blocking,
        flash::{Config, Error, Flash},
        handler,
        peripherals::Peripherals,
        time::{Duration, Instant},
        timer::{PeriodicTimer, systimer::SystemTimer},
    };
    #[cfg(multi_core)]
    use esp_hal::{
        flash::MultiCoreStrategy,
        peripherals::CPU_CTRL,
        system::{CpuControl, Stack},
    };
    use hil_test::mk_static;

    use super::*;

    const BLOCK: u32 = 0x1_0000;
    const PATTERN_WORD: u32 = 0x5AA5_C33C;

    /// The period of the probe and of core 1's ticks.
    const PERIOD: Duration = Duration::from_micros(100);
    /// The probe's runs during an operation that show it ran during the ROM call.
    ///
    /// A call that masked interrupts would allow three: one before the mask, one at the unmask for
    /// the alarms during the mask, and the next alarm, which keeps its phase and can come just
    /// after. That holds while the code around the call and the handler take less than
    /// [`PERIOD`]. They take some microseconds against 100, and the margin up to ten covers code
    /// that runs from PSRAM with cold caches. A correct run counts over a hundred.
    const PROBE_RUNS: u32 = 10;
    /// Core 1's ticks during an operation that show core 1 ran during the ROM call.
    ///
    /// Core 1 ticks [`PERIOD`] after its last tick ran, so a core 1 stopped for the call would tick
    /// at most twice: once before the call, and once when it runs again. The margin is as for
    /// [`PROBE_RUNS`].
    #[cfg(multi_core)]
    const CORE1_TICKS: u32 = 10;
    /// An operation shorter than this gives the counts too little room, and the test then says so
    /// instead of failing on a count. The time decides no pass.
    const SHORTEST: Duration = Duration::from_millis(2);

    static IN_OPERATION: AtomicBool = AtomicBool::new(false);
    static PROBE_COUNT: AtomicU32 = AtomicU32::new(0);
    static PROBE: Mutex<RefCell<Option<PeriodicTimer<'static, Blocking>>>> =
        Mutex::new(RefCell::new(None));

    // At the lowest priority, so that an operation that masks interrupts at any level stops it.
    #[handler]
    fn probe_handler() {
        if IN_OPERATION.load(Ordering::Acquire) {
            PROBE_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        critical_section::with(|cs| {
            if let Some(timer) = PROBE.borrow_ref_mut(cs).as_mut() {
                timer.clear_interrupt();
            }
        });
    }

    #[cfg(multi_core)]
    static STARTED: AtomicBool = AtomicBool::new(false);
    #[cfg(multi_core)]
    static CORE1_COUNT: AtomicU32 = AtomicU32::new(0);
    #[cfg(multi_core)]
    static CORE1_STORES: AtomicU32 = AtomicU32::new(0);

    /// The value that core 1 stores to its page on its `n`th tick.
    #[cfg(multi_core)]
    fn core1_word(n: u32) -> u32 {
        n ^ in_text()
    }

    /// Starts core 1, which ticks every [`PERIOD`], and during an operation counts its ticks and
    /// stores to `page` on each. Returns once core 1 runs.
    #[cfg(multi_core)]
    fn start_core1(cpu_ctrl: CPU_CTRL<'static>, page: Option<usize>) {
        let mut cpu_control = CpuControl::new(cpu_ctrl);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let guard = cpu_control
            .start_app_core(stack, move || {
                STARTED.store(true, Ordering::Release);
                let mut last = Instant::now();
                loop {
                    // From when the tick runs, so a core that was stopped ticks once when it runs
                    // again, not once for each period it missed.
                    if Instant::now() - last < PERIOD {
                        continue;
                    }
                    last = Instant::now();
                    if !IN_OPERATION.load(Ordering::Acquire) {
                        continue;
                    }
                    CORE1_COUNT.fetch_add(1, Ordering::Relaxed);
                    if let Some(page) = page {
                        let n = CORE1_STORES.load(Ordering::Relaxed).wrapping_add(1);
                        unsafe { (page as *mut u32).write_volatile(core1_word(n)) };
                        CORE1_STORES.store(n, Ordering::Release);
                    }
                }
            })
            .unwrap();
        core::mem::forget(guard);
        while !STARTED.load(Ordering::Acquire) {}
    }

    /// Runs `operation` once, and asserts that the probe and core 1 ran during it.
    ///
    /// `operation` must make exactly one ROM call. The bounds of the counts hold for one call: an
    /// operation of two calls would let a stopped probe run in the gap between them.
    fn assert_runs_during(operation: impl FnOnce(), what: &str) {
        PROBE_COUNT.store(0, Ordering::Relaxed);
        #[cfg(multi_core)]
        CORE1_COUNT.store(0, Ordering::Relaxed);
        let started = Instant::now();
        IN_OPERATION.store(true, Ordering::Release);
        operation();
        IN_OPERATION.store(false, Ordering::Release);
        let took = Instant::now() - started;
        let probe = PROBE_COUNT.load(Ordering::Relaxed);
        #[cfg(multi_core)]
        let core1 = CORE1_COUNT.load(Ordering::Relaxed);

        #[cfg(multi_core)]
        defmt::info!(
            "the {}: probe {}, core 1 {}, {} µs",
            what,
            probe,
            core1,
            took.as_micros()
        );
        #[cfg(not(multi_core))]
        defmt::info!("the {}: probe {}, {} µs", what, probe, took.as_micros());
        assert!(
            took >= SHORTEST || probe >= PROBE_RUNS,
            "the {what} took {took}, too short for this check"
        );
        assert!(
            probe >= PROBE_RUNS,
            "the probe ran {probe} times during the {what}"
        );
        #[cfg(multi_core)]
        assert!(
            core1 >= CORE1_TICKS,
            "core 1 ticked {core1} times during the {what}"
        );
    }

    struct Context {
        flash: Flash<'static, Blocking>,
        /// 256 KiB of PSRAM heap with a pattern, so that most of it is read back from PSRAM.
        heap: Vec<u32, ExternalMemory>,
        seed: u32,
        #[cfg(multi_core)]
        core1_page: usize,
    }

    impl Context {
        fn assert_psram_kept(&self, what: &str) {
            assert_pattern(&self.heap, self.seed, what);
            // Core 1 stores no more once the operation ends.
            #[cfg(multi_core)]
            assert_eq!(
                unsafe { (self.core1_page as *const u32).read_volatile() },
                core1_word(CORE1_STORES.load(Ordering::Acquire)),
                "core 1's last store to PSRAM, after the {what}"
            );
        }
    }

    fn setup() -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        let timer = PeriodicTimer::new(SystemTimer::new(p.SYSTIMER).alarm0);
        // Stored before it can interrupt, so the handler always finds it and clears the interrupt.
        critical_section::with(|cs| {
            let mut probe = PROBE.borrow_ref_mut(cs);
            let timer = probe.insert(timer);
            timer.set_interrupt_handler(probe_handler);
            timer.listen();
            timer.start(PERIOD).unwrap();
        });

        #[cfg(multi_core)]
        let core1_page = Box::into_raw(Box::new_in(0u32, ExternalMemory)) as usize;
        #[cfg(multi_core)]
        start_core1(p.CPU_CTRL, Some(core1_page));

        let mut heap = Vec::with_capacity_in(UNCACHED_WORDS, ExternalMemory);
        heap.resize(UNCACHED_WORDS, 0);
        let seed = fresh_seed();
        fill_pattern(&mut heap, seed);
        Context {
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
            heap,
            seed,
            #[cfg(multi_core)]
            core1_page,
        }
    }

    #[test]
    fn test_erase_and_write_run_with_interrupts() {
        let mut ctx = setup();
        let block = scratch_block(&mut ctx.flash);
        let data = mk_static!([u32; 4096], [PATTERN_WORD; 4096]);
        let flash = &mut ctx.flash;

        // SAFETY: the block is in the booted partition, after the image.
        unsafe { flash.erase(block, block + BLOCK).unwrap() };
        // SAFETY: as above.
        assert_runs_during(|| unsafe { flash.write(block, data).unwrap() }, "write");
        write_block(flash, block);
        // SAFETY: as above.
        assert_runs_during(
            || unsafe { flash.erase(block, block + BLOCK).unwrap() },
            "erase",
        );

        ctx.assert_psram_kept("write and erase");
    }

    #[test]
    fn test_reads_run_with_interrupts() {
        let mut ctx = setup();
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let app = partitions::read_partition_table(&mut ctx.flash, &mut pt_mem)
            .unwrap()
            .booted_partition()
            .unwrap()
            .unwrap()
            .offset();
        // 64 KiB, which takes milliseconds to read, so the read spans many periods.
        let buffer = mk_static!([u32; 0x4000], [0; 0x4000]);
        let flash = &mut ctx.flash;

        assert_runs_during(|| flash.read(app, buffer).unwrap(), "read");

        ctx.assert_psram_kept("read");
    }

    // A page left mapped, or its old cache lines, would return the old words to the second
    // `read_encrypted`. The new words differ from the old ones in every bit.
    #[test]
    fn test_read_encrypted_sees_new_data() {
        let mut ctx = setup();
        let block = scratch_block(&mut ctx.flash);
        let flash = &mut ctx.flash;
        let mut plain = [0u32; 256];
        let mut through_mmu = [0u32; 256];

        flash.read(block, &mut plain).unwrap();
        flash.read_encrypted(block, &mut through_mmu).unwrap();
        assert_eq!(plain, through_mmu);

        let data = plain.map(|word| !word);
        // SAFETY: the block is in the booted partition, after the image.
        unsafe {
            flash.erase(block, block + BLOCK).unwrap();
            flash.write(block, &data).unwrap();
        }
        flash.read_encrypted(block, &mut through_mmu).unwrap();
        assert_eq!(through_mmu, data);
    }

    #[cfg(multi_core)]
    #[test]
    fn test_error_strategy_allows_the_other_core() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        let config = Config::default().with_multi_core_strategy(MultiCoreStrategy::Error);
        let mut flash = Flash::new(p.FLASH, config).unwrap();
        start_core1(p.CPU_CTRL, None);

        let mut buffer = [0u32; 4];
        assert_eq!(flash.read(0x1_0000, &mut buffer), Ok(()));
    }

    #[test]
    fn test_read_into_psram_is_refused() {
        let mut ctx = setup();
        let mut buffer = Box::new_in([0u32; 4], ExternalMemory);
        assert_eq!(
            ctx.flash.read(0x1_0000, &mut buffer[..]),
            Err(Error::NotSupported)
        );
    }
}

/// Flash operations under esp-rtos, with code in PSRAM: a higher-priority thread keeps running on
/// the core that runs the operation.
///
/// The scheduler starts in this module only, because its tick interrupt would run under every other
/// test.
#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod rtos_flash {
    use core::sync::atomic::AtomicBool;

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        peripherals::Peripherals,
        time::Duration,
        timer::timg::TimerGroup,
    };
    use esp_rtos::{
        CurrentThreadHandle,
        thread::{Stack, ThreadSpawner},
    };
    use hil_test::mk_static;
    use static_cell::ConstStaticCell;

    use super::*;

    const BLOCK: u32 = 0x1_0000;
    const PATTERN_WORD: u32 = 0x5AA5_C33C;

    /// The ticker's runs during an operation that leave no doubt that one ran inside the ROM call.
    ///
    /// The ticker blocks for its period after each run, so two runs are at least one period apart.
    /// An operation that masked interrupts for its call would thus let the ticker run at most twice
    /// in it: once before the mask and once at the unmask. That holds while the code around the
    /// call takes less than the ticker's period. It takes a few microseconds against 200, and the
    /// margin up to five covers code that runs from PSRAM with cold caches. A correct run counts
    /// over a hundred.
    const INSIDE: u32 = 5;

    static IN_OPERATION: AtomicBool = AtomicBool::new(false);
    static RUNS: AtomicU32 = AtomicU32::new(0);
    static TICKER_STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());

    struct Context {
        flash: Flash<'static, Blocking>,
    }

    #[init]
    fn init() -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = TimerGroup::new(p.TIMG0);
        // Only this starts esp-rtos, on the current core. The ticker then runs only when it
        // preempts the test thread. The count depends on that.
        esp_rtos::start(timg0.timer0);
        // The ticker blocks between its runs, so this thread runs in between. Its period is far
        // below the shortest erase or write here, so it comes due many times in each.
        CurrentThreadHandle::get().set_priority(1);
        ThreadSpawner::from_static(TICKER_STACK.take())
            .with_priority(2)
            .spawn(|| {
                loop {
                    if IN_OPERATION.load(Ordering::Acquire) {
                        RUNS.fetch_add(1, Ordering::Relaxed);
                    }
                    CurrentThreadHandle::get().delay(Duration::from_micros(200));
                }
            })
            .detach();
        Context {
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
        }
    }

    /// Runs `operation`, which must make exactly one ROM call, and returns the ticker's runs during
    /// it.
    fn runs_during(operation: impl FnOnce()) -> u32 {
        RUNS.store(0, Ordering::Relaxed);
        IN_OPERATION.store(true, Ordering::Release);
        operation();
        IN_OPERATION.store(false, Ordering::Release);
        RUNS.load(Ordering::Relaxed)
    }

    #[test]
    fn test_a_higher_priority_thread_runs_during_an_erase(mut ctx: Context) {
        let block = scratch_block(&mut ctx.flash);
        let flash = &mut ctx.flash;
        write_block(flash, block);
        // SAFETY: the block is in the booted partition, after the image.
        let runs = runs_during(|| unsafe { flash.erase(block, block + BLOCK).unwrap() });
        assert!(
            runs >= INSIDE,
            "the ticker ran {runs} times during the erase"
        );
        defmt::info!("the ticker ran {} times during the erase", runs);
    }

    #[test]
    fn test_a_higher_priority_thread_runs_during_a_write(mut ctx: Context) {
        let block = scratch_block(&mut ctx.flash);
        let data = mk_static!([u32; 4096], [PATTERN_WORD; 4096]);
        let flash = &mut ctx.flash;
        // SAFETY: the block is in the booted partition, after the image.
        unsafe { flash.erase(block, block + BLOCK).unwrap() };
        // SAFETY: as above.
        let runs = runs_during(|| unsafe { flash.write(block, data).unwrap() });
        assert!(
            runs >= INSIDE,
            "the ticker ran {runs} times during the write"
        );
        defmt::info!("the ticker ran {} times during the write", runs);
    }
}

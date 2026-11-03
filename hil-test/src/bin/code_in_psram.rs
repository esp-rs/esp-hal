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

// 256 KiB, so the static spans several MMU pages.
#[esp_hal::psram(zeroed)]
static mut PSRAM_BSS: [u32; 0x1_0000] = [0; 0x1_0000];

struct PsramState {
    waker: Mutex<RefCell<Option<core::task::Waker>>>,
    flag: core::sync::atomic::AtomicBool,
}

#[esp_hal::psram(zeroed)]
static PSRAM_STATE: PsramState = PsramState {
    waker: Mutex::new(RefCell::new(None)),
    flag: core::sync::atomic::AtomicBool::new(false),
};

#[repr(C, align(64))]
struct DmaBlock([u8; 4096]);

#[esp_hal::psram(zeroed)]
static mut DMA_SOURCE: DmaBlock = DmaBlock([0; 4096]);

unsafe extern "C" {
    static _psram_code_end: u8;
    static _psram_bss_end: u8;
}

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

/// How often a test that must observe a flash operation in progress tries again. Each attempt is
/// one operation, and the count only bounds the run time of a failing test.
#[cfg(multi_core)]
const ATTEMPTS: u32 = 8;

// The two-core tests read the lock.
#[cfg(multi_core)]
unsafe extern "Rust" {
    /// esp-hal's flash chip lock: 0 while free, otherwise one more than the number of the core that
    /// holds it, for a ROM flash call, an encrypted read, or while it parks or stops the other
    /// core. Every copy of esp-hal finds it by this name.
    static ESP_HAL_FLASH_CHIP_LOCK: portable_atomic::AtomicU8;
}

/// Returns the value of esp-hal's flash chip lock.
#[cfg(multi_core)]
fn flash_chip_holder() -> u8 {
    // SAFETY: esp-hal defines the static.
    unsafe { ESP_HAL_FLASH_CHIP_LOCK.load(Ordering::Acquire) }
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
        // The heap starts at the page after the code and the statics, as the linker placed them.
        let used = (addr_of!(_psram_code_end) as u32).max(addr_of!(_psram_bss_end) as u32);
        assert_eq!(
            start as u32,
            used.next_multiple_of(property!("mmu.page_size"))
        );

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

    // PSRAM keeps its content across a reset, so the zeros come from the start-up, at every boot.
    #[test]
    fn test_psram_static_is_zeroed_above_the_code() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        let start = addr_of!(PSRAM_BSS) as u32;
        assert!(in_window(start));
        assert!(start >= addr_of!(_psram_code_end) as u32);

        let words = unsafe { &mut *addr_of_mut!(PSRAM_BSS) };
        assert_eq!(words.iter().position(|&w| w != 0), None);
        fill_pattern(words, PATTERN);
        assert_pattern(words, PATTERN, "PSRAM_BSS");
    }

    #[test]
    fn test_psram_static_holds_none_and_false() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(addr_of!(PSRAM_STATE) as u32));
        assert!(critical_section::with(|cs| PSRAM_STATE
            .waker
            .borrow_ref(cs)
            .is_none()));
        assert!(!PSRAM_STATE.flag.load(Ordering::Relaxed));
        PSRAM_STATE.flag.store(true, Ordering::Relaxed);
        assert!(PSRAM_STATE.flag.load(Ordering::Relaxed));
    }

    // The CPU writes the source through the cache just before the transfer, so the DMA reads the
    // written bytes only if the driver treats the static as PSRAM and writes the cache back.
    // Outside `psram_range`, the driver rejects the buffer.
    #[cfg(all(dma_can_access_psram, dma_supports_mem2mem))]
    #[test]
    fn test_dma_reads_psram_static() {
        use esp_hal::dma::{BurstConfig, ExternalBurstConfig, Mem2Mem};

        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mem2mem = cfg_select! {
            feature = "esp32s31" => Mem2Mem::new(peripherals.DMA_AXI_CH1),
            _ => Mem2Mem::new(peripherals.DMA_CH0),
        };
        let (rx_buffer, rx_descriptors, _, tx_descriptors) = esp_hal::dma_buffers!(4096);
        let mut mem2mem = mem2mem
            .with_descriptors(
                rx_descriptors,
                tx_descriptors,
                BurstConfig {
                    external_memory: ExternalBurstConfig::Size64,
                    internal_memory: Default::default(),
                },
            )
            .unwrap();

        let source = unsafe { &mut (*addr_of_mut!(DMA_SOURCE)).0 };
        assert!(in_window(source.as_ptr() as u32));
        for (i, byte) in source.iter_mut().enumerate() {
            *byte = (i as u8).wrapping_mul(31).wrapping_add(7);
        }

        mem2mem
            .start_transfer(rx_buffer, source)
            .unwrap()
            .wait()
            .unwrap();

        for (i, &byte) in rx_buffer.iter().enumerate() {
            assert_eq!(byte, (i as u8).wrapping_mul(31).wrapping_add(7));
        }
    }

    #[test]
    fn test_heap_after_psram_statics() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());

        let (start, _) = psram.raw_parts();
        assert!(start as u32 >= addr_of!(_psram_bss_end) as u32);
        assert!(start as u32 >= addr_of!(PSRAM_BSS) as u32 + 0x4_0000);
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

/// A stop or a park of the APP core that starts while the APP core is inside the ROM call of an
/// erase. Both must wait until the call ends, so afterwards no core holds the flash chip.
///
/// The APP core writes and erases its block in a loop. Its own timer handler, on seeing the erase
/// in progress, holds the APP core inside the ROM call until the PRO core releases it. The PRO core
/// starts the stop or the park only once the APP core is held, so the erase is in progress when the
/// stop or the park starts, whatever its length. A PRO-core timer releases the APP core while the
/// stop or the park waits. The flash chip lock, read after the stop or the park, shows that the APP
/// core left the ROM call before it stopped or parked. The APP core's timer keeps running, so its
/// handler's runs show whether the APP core runs.
#[cfg(multi_core)]
#[embedded_test::tests(default_timeout = 30)]
mod stop_and_park_during_erase {
    use core::{ptr::addr_of_mut, sync::atomic::AtomicBool};

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        handler,
        peripherals::{FLASH, Peripherals, TIMG1},
        system::{AppCoreGuard, Cpu, CpuControl, Stack},
        time::{Duration, Instant},
        timer::{OneShotTimer, PeriodicTimer, timg::TimerGroup},
    };
    use hil_test::mk_static;

    use super::*;

    const BLOCK: u32 = 0x1_0000;
    /// The period of the APP core's timer.
    const APP_PERIOD: Duration = Duration::from_micros(200);
    /// A hang detector for the APP core after the park ends: its next erase follows a block
    /// write.
    const RESUME_BOUND: Duration = Duration::from_secs(4);
    /// The PRO core releases the APP core this long after it starts the stop or the park. A stop
    /// that does not take the flash chip lock posts its park request well within it, and then
    /// parks the APP core inside the ROM call. Any value passes a correct stop.
    const RELEASE_AFTER: Duration = Duration::from_millis(1);

    const WRITE: u32 = 0;
    const ERASE: u32 = 1;

    static PHASE: AtomicU32 = AtomicU32::new(WRITE);
    static ERASES: AtomicU32 = AtomicU32::new(0);
    static ERASE_FAILED: AtomicBool = AtomicBool::new(false);
    static HOLD: AtomicBool = AtomicBool::new(false);
    static HELD: AtomicBool = AtomicBool::new(false);
    static RELEASE: AtomicBool = AtomicBool::new(false);
    static RESUMED: AtomicBool = AtomicBool::new(false);
    /// The runs of the APP core's timer handler.
    static APP_RUNS: AtomicU32 = AtomicU32::new(0);

    /// The APP core's timer. The APP core stores it before it starts it, and then only the handler
    /// uses it.
    static mut APP_TIMER: Option<PeriodicTimer<'static, Blocking>> = None;
    /// The PRO core's release timer, used in the same way.
    static mut RELEASE_TIMER: Option<OneShotTimer<'static, Blocking>> = None;

    /// The flash chip lock's value while the APP core holds it.
    const APP_HOLDS: u8 = Cpu::AppCpu as u8 + 1;

    // Runs on the APP core. The APP core holds the flash chip lock only around the ROM call of its
    // operation, so a handler that finds it held in the erase phase interrupted the erase there.
    #[handler]
    fn hold_in_erase() {
        // SAFETY: after the APP core starts the timer, only this handler uses `APP_TIMER`.
        let timer = unsafe { (*addr_of_mut!(APP_TIMER)).as_mut() }.unwrap();
        timer.clear_interrupt();
        APP_RUNS.fetch_add(1, Ordering::Release);
        if !HOLD.load(Ordering::Acquire)
            || HELD.load(Ordering::Relaxed)
            || PHASE.load(Ordering::Acquire) != ERASE
            || flash_chip_holder() != APP_HOLDS
        {
            return;
        }
        HELD.store(true, Ordering::Release);
        while !RELEASE.load(Ordering::Acquire) {}
        // Stored before the ROM call can end. A stop or a park that returns without it did not
        // wait for the APP core at all.
        RESUMED.store(true, Ordering::Release);
    }

    #[handler]
    fn release() {
        // SAFETY: after the test arms the timer, only this handler uses `RELEASE_TIMER`.
        if let Some(timer) = unsafe { (*addr_of_mut!(RELEASE_TIMER)).as_mut() } {
            timer.clear_interrupt();
        }
        RELEASE.store(true, Ordering::Release);
    }

    struct Context {
        flash: Flash<'static, Blocking>,
        app_flash: FLASH<'static>,
        timg1: TIMG1<'static>,
        cpu_control: CpuControl<'static>,
    }

    fn setup() -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = TimerGroup::new(p.TIMG0);
        let mut timer = OneShotTimer::new(timg0.timer0);
        timer.set_interrupt_handler(release);
        // SAFETY: the timer is not armed yet, so the handler does not use `RELEASE_TIMER`.
        unsafe { (*addr_of_mut!(RELEASE_TIMER)).insert(timer) }.listen();
        Context {
            // SAFETY: each core uses its own `Flash`, and the flash chip lock keeps their ROM
            // calls apart.
            app_flash: unsafe { p.FLASH.clone_unchecked() },
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
            timg1: p.TIMG1,
            cpu_control: CpuControl::new(p.CPU_CTRL),
        }
    }

    /// Starts the APP core, which writes and erases `block` in a loop with its own `Flash`, and
    /// returns once its timer handler holds it inside an erase.
    fn start_and_hold<'a>(
        cpu_control: &mut CpuControl<'static>,
        app_flash: FLASH<'static>,
        timg1: TIMG1<'static>,
        block: u32,
    ) -> AppCoreGuard<'a> {
        let stack = mk_static!(Stack<8192>, Stack::new());
        let guard = cpu_control
            .start_app_core(stack, move || {
                let mut flash = Flash::new(app_flash, Config::default()).unwrap();
                // The APP core binds the handler, so the handler runs on the APP core.
                let mut timer = PeriodicTimer::new(TimerGroup::new(timg1).timer0);
                timer.set_interrupt_handler(hold_in_erase);
                // SAFETY: the timer does not run yet, so the handler does not use `APP_TIMER`.
                let timer = unsafe { (*addr_of_mut!(APP_TIMER)).insert(timer) };
                timer.listen();
                timer.start(APP_PERIOD).unwrap();
                loop {
                    write_block(&mut flash, block);
                    PHASE.store(ERASE, Ordering::Release);
                    // SAFETY: the block is in the booted partition, after the image.
                    if unsafe { flash.erase(block, block + BLOCK) }.is_err() {
                        ERASE_FAILED.store(true, Ordering::Relaxed);
                    }
                    PHASE.store(WRITE, Ordering::Release);
                    ERASES.fetch_add(1, Ordering::Release);
                }
            })
            .unwrap();
        HOLD.store(true, Ordering::Release);
        while !HELD.load(Ordering::Acquire) {
            assert!(
                ERASES.load(Ordering::Acquire) < ATTEMPTS,
                "the APP core's timer never held it inside an erase, in {ATTEMPTS} erases"
            );
        }
        guard
    }

    /// Arms the release of the APP core.
    fn release_soon() {
        // SAFETY: the timer is not armed, so the handler does not use `RELEASE_TIMER`.
        let timer = unsafe { (*addr_of_mut!(RELEASE_TIMER)).as_mut() }.unwrap();
        timer.schedule(RELEASE_AFTER).unwrap();
    }

    /// Checks the state after a stop or a park of the APP core, which held the flash chip lock
    /// when the stop or the park started.
    fn assert_outside_the_erase(action: &str) {
        assert!(
            RESUMED.load(Ordering::Acquire),
            "the {action} returned before the APP core left its timer handler"
        );
        assert_eq!(
            flash_chip_holder(),
            0,
            "the {action} left the APP core holding the flash chip"
        );
        assert!(!ERASE_FAILED.load(Ordering::Relaxed), "an erase failed");
    }

    /// Asserts that the APP core's timer handler does not run for `periods` of its periods. A
    /// stopped or parked core runs nothing, so any count of periods passes a correct stop or park.
    /// The count sets only how surely a core that still runs shows.
    fn assert_app_core_idle(periods: u32, action: &str) {
        let runs = APP_RUNS.load(Ordering::Acquire);
        let end = Instant::now() + APP_PERIOD * periods;
        while Instant::now() < end {}
        assert_eq!(
            APP_RUNS.load(Ordering::Acquire),
            runs,
            "the APP core ran after the {action}"
        );
    }

    #[test]
    fn test_stop_waits_for_the_app_core_erase() {
        let mut ctx = setup();
        let block = scratch_block(&mut ctx.flash);

        let guard = start_and_hold(&mut ctx.cpu_control, ctx.app_flash, ctx.timg1, block);
        let before = Instant::now();
        release_soon();
        drop(guard);
        defmt::info!("the stop took {} µs", (Instant::now() - before).as_micros());

        assert_outside_the_erase("stop");
        assert_app_core_idle(50, "stop");
        // The PRO core can use the flash: a stuck lock would hang here, and an operation cut
        // short would show in the read-backs. The block held the pattern, so the first read-back
        // shows the erase.
        // SAFETY: the block is in the booted partition, after the image.
        unsafe { ctx.flash.erase(block, block + BLOCK).unwrap() };
        assert_block(&mut ctx.flash, block, |_| u32::MAX, "erased");
        write_block(&mut ctx.flash, block);
        // SAFETY: nothing writes `BLOCK_PATTERN`.
        let pattern = unsafe { &*core::ptr::addr_of!(BLOCK_PATTERN) };
        assert_block(
            &mut ctx.flash,
            block,
            |i| pattern[i % pattern.len()],
            "written",
        );
    }

    /// Reads the block back, and asserts that word `i` is `expected(i)`.
    fn assert_block(
        flash: &mut Flash<'static, Blocking>,
        block: u32,
        expected: impl Fn(usize) -> u32,
        what: &str,
    ) {
        let mut words = [0u32; 1024];
        for offset in (0..BLOCK).step_by(core::mem::size_of_val(&words)) {
            flash.read(block + offset, &mut words).unwrap();
            let first = offset as usize / 4;
            let difference = (0..words.len()).find(|&i| words[i] != expected(first + i));
            assert_eq!(
                difference,
                None,
                "the block is not {what} at {:#x}",
                block + offset
            );
        }
    }

    #[test]
    fn test_park_waits_for_the_app_core_erase() {
        let mut ctx = setup();
        let block = scratch_block(&mut ctx.flash);

        let _running = start_and_hold(&mut ctx.cpu_control, ctx.app_flash, ctx.timg1, block);
        let before = Instant::now();
        release_soon();
        let parked = ctx.cpu_control.park_core(Cpu::AppCpu);
        defmt::info!("the park took {} µs", (Instant::now() - before).as_micros());

        assert_outside_the_erase("park");
        assert_app_core_idle(50, "park");

        let erases = ERASES.load(Ordering::Acquire);
        drop(parked);
        let resumed = Instant::now();
        while ERASES.load(Ordering::Acquire) == erases {
            assert!(
                Instant::now() - resumed < RESUME_BOUND,
                "the APP core did not erase after the park"
            );
        }
    }
}

/// A light sleep that a PRO-core interrupt requests inside the ROM call of the PRO core's erase,
/// while the APP core waits for the flash with its interrupts masked. The sleep parks the APP core
/// before it checks the flash, so the APP core must answer the park while it waits, and the sleep
/// then returns `Refused`.
///
/// The test observes the check from inside the sleep: `sleep_light_if` calls its closure only
/// after the flash check let the sleep through, with the APP core parked. The closure refuses
/// every sleep, so the chip never sleeps, even if the flash check is broken, and the debug probe
/// stays connected.
#[cfg(multi_core)]
#[embedded_test::tests(default_timeout = 30)]
mod sleep_during_erase_with_masked_reader {
    use core::{ptr::addr_of_mut, sync::atomic::AtomicBool};

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        handler,
        peripherals::{LPWR, Peripherals},
        rtc_cntl::sleep::{LightSleep, LowPower, RtcSleepConfig},
        system::{Cpu, CpuControl, Stack},
        time::{Duration, Instant},
        timer::{PeriodicTimer, timg::TimerGroup},
    };
    use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
    use hil_test::mk_static;

    use super::*;

    const BLOCK: u32 = 0x1_0000;

    const NOT_REQUESTED: u32 = 0;
    const REFUSED: u32 = 1;
    const NOT_REFUSED: u32 = 2;

    static ERASING: AtomicBool = AtomicBool::new(false);
    static STOP_READS: AtomicBool = AtomicBool::new(false);
    static READS: AtomicU32 = AtomicU32::new(0);
    static READ_FAILED: AtomicBool = AtomicBool::new(false);
    /// Whether the APP core reads with its interrupts masked.
    static MASKED: AtomicBool = AtomicBool::new(false);
    /// Per attempt: the result of the sleep, whether the closure ran and found the flash chip lock
    /// held, and whether the APP core still read masked when the sleep returned.
    static SLEEP_RESULT: AtomicU32 = AtomicU32::new(NOT_REQUESTED);
    static CLOSURE_RAN: AtomicBool = AtomicBool::new(false);
    static HELD_IN_CLOSURE: AtomicBool = AtomicBool::new(false);
    static MASKED_AFTER: AtomicBool = AtomicBool::new(false);

    /// The timer of the sleep. The test stores it before it starts it, and then only the handler
    /// uses it.
    static mut TIMER: Option<PeriodicTimer<'static, Blocking>> = None;

    /// The flash chip lock's value while the PRO core holds it.
    const PRO_HOLDS: u8 = Cpu::ProCpu as u8 + 1;

    // The light sleep calls this only after its flash check found the lock free, with the APP core
    // parked (`sleep_with_interrupts_disabled`). The test relies on that order.
    fn may_sleep() -> bool {
        CLOSURE_RAN.store(true, Ordering::Relaxed);
        HELD_IN_CLOSURE.store(flash_chip_holder() != 0, Ordering::Relaxed);
        false
    }

    // The PRO core holds the flash chip lock only around the erase's ROM call, so a handler that
    // finds it held interrupted the erase there. `MASKED` shows the APP core masked.
    #[handler]
    fn sleep_in_erase() {
        // SAFETY: after the test starts the timer, only this handler uses `TIMER`.
        if let Some(timer) = unsafe { (*addr_of_mut!(TIMER)).as_mut() } {
            timer.clear_interrupt();
        }
        if SLEEP_RESULT.load(Ordering::Relaxed) != NOT_REQUESTED
            || !ERASING.load(Ordering::Acquire)
            || flash_chip_holder() != PRO_HOLDS
            || !MASKED.load(Ordering::Acquire)
        {
            return;
        }
        let mut lpwr = LowPower::new(unsafe { LPWR::steal() });
        let result = lpwr.sleep_light_if(RtcSleepConfig::default(), may_sleep);
        MASKED_AFTER.store(MASKED.load(Ordering::Acquire), Ordering::Relaxed);
        let code = if result == LightSleep::Refused {
            REFUSED
        } else {
            NOT_REFUSED
        };
        SLEEP_RESULT.store(code, Ordering::Release);
    }

    #[test]
    fn test_sleep_is_refused_while_the_other_core_waits_masked() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        // SAFETY: each core uses its own `Flash`, and the flash chip lock keeps their ROM calls
        // apart.
        let app_flash = unsafe { p.FLASH.clone_unchecked() };
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let stack = mk_static!(Stack<8192>, Stack::new());
        let _app = cpu_control
            .start_app_core(stack, move || {
                let mut flash = Flash::new(app_flash, Config::default()).unwrap();
                let mut words = [0u32; 64];
                while !STOP_READS.load(Ordering::Acquire) {
                    // The read waits for the PRO core's erase with this core's interrupts masked. A
                    // critical section would be held across the park, which the flash module's
                    // limits forbid.
                    // SAFETY: the token is released below, on this core.
                    let token = unsafe { SingleCoreInterruptLock.enter() };
                    MASKED.store(true, Ordering::Release);
                    let ok = flash.read(block, &mut words).is_ok();
                    MASKED.store(false, Ordering::Release);
                    // SAFETY: the token comes from the `enter` above.
                    unsafe { SingleCoreInterruptLock.exit(token) };
                    if !ok {
                        READ_FAILED.store(true, Ordering::Relaxed);
                    }
                    READS.fetch_add(1, Ordering::Release);
                }
            })
            .unwrap();
        while READS.load(Ordering::Acquire) == 0 {}

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut timer = PeriodicTimer::new(timg0.timer0);
        timer.set_interrupt_handler(sleep_in_erase);
        // SAFETY: the timer does not run yet, so the handler does not use `TIMER`.
        let timer = unsafe { (*addr_of_mut!(TIMER)).insert(timer) };
        timer.listen();

        let mut passed = false;
        for _ in 0..ATTEMPTS {
            SLEEP_RESULT.store(NOT_REQUESTED, Ordering::Relaxed);
            CLOSURE_RAN.store(false, Ordering::Relaxed);
            HELD_IN_CLOSURE.store(false, Ordering::Relaxed);
            MASKED_AFTER.store(false, Ordering::Relaxed);
            write_block(&mut flash, block);

            ERASING.store(true, Ordering::Release);
            timer.start(Duration::from_micros(200)).unwrap();
            // SAFETY: the block is in the booted partition, after the image.
            let erased = unsafe { flash.erase(block, block + BLOCK) }.is_ok();
            timer.cancel().unwrap();
            ERASING.store(false, Ordering::Release);
            assert!(erased, "the erase failed");

            let result = SLEEP_RESULT.load(Ordering::Acquire);
            let closure_ran = CLOSURE_RAN.load(Ordering::Relaxed);
            assert!(
                !(closure_ran && HELD_IN_CLOSURE.load(Ordering::Relaxed)),
                "the flash check let a light sleep through while a core held the flash chip"
            );
            // The closure refuses every sleep it sees.
            assert!(
                result != NOT_REFUSED,
                "the light sleep was neither refused by the flash check nor by the closure"
            );
            // No request, or the APP core no longer read masked: try again.
            if result == REFUSED && !closure_ran && MASKED_AFTER.load(Ordering::Relaxed) {
                passed = true;
                break;
            }
        }
        timer.unlisten();

        let reads = READS.load(Ordering::Acquire);
        let deadline = Instant::now() + Duration::from_secs(1);
        while READS.load(Ordering::Acquire) == reads {
            assert!(
                Instant::now() < deadline,
                "the APP core read nothing after the erase"
            );
        }
        STOP_READS.store(true, Ordering::Release);

        assert!(
            !READ_FAILED.load(Ordering::Relaxed),
            "a read on the APP core failed"
        );
        assert!(
            passed,
            "no light sleep was refused by the flash check with the APP core masked, in {ATTEMPTS} erases"
        );
    }
}

//! Code in PSRAM: the firmware's code and read-only data run from the PSRAM window.

//% CHIP_FILTER: psram_supports_code_in_psram
//% ENV: ESP_HAL_CONFIG_CODE_IN_PSRAM=true
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

static SWINT: Mutex<RefCell<Option<SoftwareInterrupt<'static, 2>>>> =
    Mutex::new(RefCell::new(None));
static INTERRUPTS: AtomicU32 = AtomicU32::new(0);

// 256 KiB, so the static spans several MMU pages.
#[esp_hal::psram(zeroed)]
static mut PSRAM_BSS: [u32; 0x1_0000] = [0; 0x1_0000];

// Not `mut`: the attribute accepts a static with interior mutability.
#[esp_hal::psram(zeroed)]
static PSRAM_WAKER: Mutex<RefCell<Option<core::task::Waker>>> = Mutex::new(RefCell::new(None));

#[repr(C, align(64))]
struct DmaBlock([u8; 4096]);

#[esp_hal::psram(zeroed)]
static mut DMA_SOURCE: DmaBlock = DmaBlock([0; 4096]);

unsafe extern "C" {
    static _psram_code_end: u8;
    static _psram_bss_end: u8;
}

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

/// Calls `f` with the load address, the length, and the flash offset of the data of each segment
/// of the booted image. Returns the offset and the length of the booted partition, and the flash
/// offset after the last segment.
fn booted_image(
    flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>,
    mut f: impl FnMut(&mut esp_hal::flash::Flash<'_, esp_hal::Blocking>, u32, u32, u32),
) -> (u32, u32, u32) {
    use esp_bootloader_esp_idf::partitions;

    let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let pt = partitions::read_partition_table(flash, &mut pt_mem).unwrap();
    let app = pt.booted_partition().unwrap().unwrap();
    let (app, app_len) = (app.offset(), app.len());

    // ESP-IDF image: a 24-byte header, then segments of load address, length, and data.
    let mut header = [0u32; 6];
    flash.read(app, &mut header).unwrap();
    assert_eq!(header[0] & 0xFF, 0xE9);
    let mut offset = app + 24;
    for _ in 0..(header[0] >> 8) & 0xFF {
        let mut segment = [0u32; 2];
        flash.read(offset, &mut segment).unwrap();
        let [load, len] = segment;
        f(flash, load, len, offset + 8);
        offset += 8 + len;
    }
    (app, app_len, offset)
}

const BLOCK: u32 = 0x1_0000;
const SECTOR: u32 = 0x1000;

/// Returns the flash offset of a random 64 KiB block after the image in the booted partition. A
/// random block spreads the wear of the tests over the partition.
fn scratch_block(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>) -> u32 {
    // The checksum and the hash after the last segment fit in the next block.
    let (app, app_len, end) = booted_image(flash, |_, _, _, _| {});
    let first = (end + BLOCK).div_ceil(BLOCK);
    let last = (app + app_len) / BLOCK;
    assert!(
        first < last,
        "no free 64 KiB block after the image in the booted partition"
    );
    let block = (first + esp_hal::rng::Rng::new().random() % (last - first)) * BLOCK;
    defmt::info!("scratch block at {:#x}", block);
    block
}

/// The flash operations that a test tries before it fails.
#[cfg(multi_core)]
const ATTEMPTS: u32 = 8;

/// The pattern for [`write_sectors`]. A nonzero `static mut` is in internal RAM, where a ROM flash
/// routine can read it.
static mut SECTOR_PATTERN: [u32; SECTOR as usize / 4] = [0x5AA5_C33C; SECTOR as usize / 4];

/// Writes the pattern over the 15 sectors of [`erase_sectors`], one sector in each flash operation.
fn write_sectors(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>, block: u32) {
    // SAFETY: nothing writes `SECTOR_PATTERN`.
    let pattern = unsafe { &*core::ptr::addr_of!(SECTOR_PATTERN) };
    for sector in 1..16 {
        // SAFETY: the callers pass a block in the booted partition, after the image.
        unsafe { flash.write(block + sector * SECTOR, pattern) }.unwrap();
    }
}

/// Erases the 15 sectors after the first sector of the 64 KiB block at `block`, in one flash
/// operation. The range does not start at a block, so the operation is 15 ROM calls.
fn erase_sectors(
    flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>,
    block: u32,
) -> Result<(), esp_hal::flash::Error> {
    // SAFETY: the callers pass a block in the booted partition, after the image.
    unsafe { flash.erase(block + SECTOR, block + 16 * SECTOR) }
}

fn sectors_erased(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>, block: u32) -> bool {
    let mut words = [0u32; SECTOR as usize / 4];
    (1..16).all(|sector| {
        flash.read(block + sector * SECTOR, &mut words).is_ok()
            && words.iter().all(|&word| word == u32::MAX)
    })
}

/// The number of `u32` words in 256 KiB, the size of the PSRAM data that the tests write and read
/// back. It is larger than the data cache, so most of the read-back comes from PSRAM.
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

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use core::ptr::{addr_of, addr_of_mut};

    #[cfg(esp32s31)]
    use esp_hal::{
        clock::{
            ClockConfig,
            ll::{
                ClockTree,
                MpllClkConfig,
                mpll_clk_frequency,
                release_mpll_clk,
                request_mpll_clk,
            },
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

    /// Writes a distinct word at offset 0 and at each power-of-two word offset of the `words` at
    /// `start`, pushes them out of the cache, and reads them back. Fails if PSRAM is smaller than
    /// the range: two of the offsets then reach the same PSRAM word.
    fn assert_no_alias(start: *mut u32, words: usize) {
        // The power-of-two offsets are below 768 KiB or from 1 MiB up, so the pattern does not
        // overwrite them.
        const PUSH_OUT: usize = 0xC_0000 / 4;
        assert!(words >= PUSH_OUT + UNCACHED_WORDS);
        let heap = unsafe { core::slice::from_raw_parts_mut(start, words) };
        let offsets =
            || core::iter::once(0).chain((0..).map(|bit| 1 << bit).take_while(|&i| i < words));
        for i in offsets() {
            heap[i] = PATTERN ^ i as u32;
        }
        fill_pattern(&mut heap[PUSH_OUT..][..UNCACHED_WORDS], fresh_seed());
        let wrong = offsets().find(|&i| heap[i] != PATTERN ^ i as u32);
        assert_eq!(
            wrong, None,
            "the word at this offset differs: PSRAM is smaller than the heap"
        );
    }

    #[test]
    fn test_code_and_rodata_in_psram() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());

        assert!(in_window(in_text as *const () as u32));
        assert!(in_window(addr_of!(RODATA) as u32));
        assert_eq!(in_text(), 0x5053_524D);
    }

    // Reads go through the data cache, so this test shows the content and bounds of the copy, not
    // the write-back.
    #[test]
    #[timeout(30)]
    fn test_segments_match_image() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();

        let mut compared = 0;
        booted_image(&mut flash, |flash, load, len, data| {
            if !in_window(load) {
                return;
            }
            let mut chunk = [0u32; 256];
            let mut done = 0;
            while done < len {
                let words = ((len - done) / 4).min(chunk.len() as u32) as usize;
                flash.read(data + done, &mut chunk[..words]).unwrap();
                let mapped =
                    unsafe { core::slice::from_raw_parts((load + done) as *const u32, words) };
                let first_difference = chunk[..words].iter().zip(mapped).position(|(a, b)| a != b);
                assert_eq!(first_difference, None, "in the 1 KiB at {:#x}", load + done);
                done += words as u32 * 4;
            }
            compared += len;
        });
        assert!(compared > 0);
    }

    #[test]
    fn test_heap_after_code() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());

        let (start, size) = psram.raw_parts();
        // The heap starts at the page after the code and the statics, as the linker placed them.
        let used = (addr_of!(_psram_code_end) as u32).max(addr_of!(_psram_bss_end) as u32);
        assert_eq!(
            start as u32,
            used.next_multiple_of(property!("mmu.page_size"))
        );
        // `Psram::size` is the whole chip.
        assert!(psram.size().is_power_of_two());

        let words = size / 4;
        assert_no_alias(start as *mut u32, words);
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, words) };
        // Fails with an access fault if the mapping ends before the range does.
        heap[words - 1] = PATTERN;
        assert_eq!(heap[words - 1], PATTERN);
    }

    /// Asserts that `PSRAM_BSS` is all zero and above the code, then writes a pattern over all of
    /// it.
    ///
    /// PSRAM keeps its content across the reset between two tests. The static is larger than the
    /// cache, so most of the pattern is in PSRAM at the reset. The second test that calls this
    /// function reads zeros only if the start-up zeroed the static.
    fn assert_psram_bss_zeroed_then_write() {
        let start = addr_of!(PSRAM_BSS) as u32;
        assert!(in_window(start));
        assert!(start >= addr_of!(_psram_code_end) as u32);

        let words = unsafe { &mut *addr_of_mut!(PSRAM_BSS) };
        assert_eq!(words.iter().position(|&w| w != 0), None);
        fill_pattern(words, PATTERN);
    }

    #[test]
    fn test_psram_static_is_zeroed_above_the_code() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        assert_psram_bss_zeroed_then_write();
        assert!(critical_section::with(|cs| PSRAM_WAKER
            .borrow_ref(cs)
            .is_none()));
    }

    #[test]
    fn test_psram_static_is_zeroed_again() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        assert_psram_bss_zeroed_then_write();
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
            esp32s31 => Mem2Mem::new(peripherals.DMA_AXI_CH1),
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

    // The heap and a PSRAM static keep their content over a flash erase, a write and a read.
    #[test]
    fn test_psram_survives_flash_operations() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(peripherals);
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);

        let (start, _) = psram.raw_parts();
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, UNCACHED_WORDS) };
        let seed = fresh_seed();
        fill_pattern(heap, seed);
        let words = unsafe { &mut *addr_of_mut!(PSRAM_BSS) };
        fill_pattern(words, PATTERN);

        let data = [PATTERN; 64];
        unsafe {
            flash.erase(block, block + BLOCK).unwrap();
            flash.write(block, &data).unwrap();
        }
        let mut buffer = [0u32; 4096];
        flash.read(block, &mut buffer).unwrap();

        assert_pattern(heap, seed, "the heap after the flash operations");
        assert_pattern(words, PATTERN, "PSRAM_BSS after the flash operations");
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

    /// The MPLL rate in the hardware: 40 MHz × (feedback divider + 1) / 2.
    #[cfg(esp32s31)]
    fn mpll_in_hardware() -> u32 {
        let fb_div = esp_hal::peripherals::LP_AON_CLK_RST::regs()
            .mspi_div()
            .read()
            .mspi_fb_div()
            .bits();
        (u32::from(fb_div) + 1) * 40_000_000 / 2
    }

    /// The PSRAM bus clock in the hardware: the MPLL rate over the bus clock divider. Fails if
    /// PSRAM does not run from the MPLL.
    #[cfg(esp32s31)]
    fn psram_in_hardware() -> u32 {
        let source = esp_hal::peripherals::HP_SYS_CLKRST::regs()
            .psram_ctrl0()
            .read()
            .clk_src_sel()
            .bits();
        assert_eq!(source, 1, "PSRAM does not run from the MPLL");
        let bits = esp_hal::peripherals::MEMSPI2::regs()
            .sram_clk()
            .read()
            .bits();
        mpll_in_hardware() / ((bits & 0xff) + 1)
    }

    /// Asserts that the clock tree and the hardware both run the MPLL at `mpll_hz`, and PSRAM at
    /// half of it.
    #[cfg(esp32s31)]
    fn assert_rates(mpll: MpllClkConfig, mpll_hz: u32) {
        assert_eq!(ClockTree::with(|clocks| clocks.mpll_clk()), Some(mpll));
        assert_eq!(mpll_clk_frequency(), mpll_hz);
        assert_eq!(mpll_in_hardware(), mpll_hz);
        assert_eq!(psram_in_hardware(), mpll_hz / 2);
    }

    /// Erases, writes and reads back a sector after the image. Fails if a flash operation fails
    /// or the sector does not hold what was written.
    #[cfg(esp32s31)]
    fn assert_flash_works(flash: &mut Flash<'_, esp_hal::Blocking>, seed: u32) {
        let sector = scratch_block(flash);
        let mut written = [0u32; 1024];
        fill_pattern(&mut written, seed);
        // SAFETY: the sector is the first of a block in the booted partition, after the image.
        unsafe {
            flash.erase(sector, sector + Flash::SECTOR_SIZE).unwrap();
            flash.write(sector, &written).unwrap();
        }
        let mut read_back = [0u32; 1024];
        flash.read(sector, &mut read_back).unwrap();
        assert_pattern(&read_back, seed, "the flash sector");
    }

    // 500 MHz in `ClockConfig::mpll_clk` moves PSRAM from 200 to 250 MHz in `init`, while the code
    // runs from PSRAM. Fails if the clock tree and the hardware disagree, or if the code, the heap
    // or flash fail at the new rate.
    #[cfg(esp32s31)]
    #[test]
    fn test_mpll_rate_500mhz() {
        let mut clocks = ClockConfig::from(esp_hal::clock::CpuClock::default());
        clocks.mpll_clk = Some(MpllClkConfig::_500);
        let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clocks));
        hil_test::init_logger!(peripherals);

        assert_rates(MpllClkConfig::_500, 500_000_000);
        let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
        let (start, _) = psram.raw_parts();
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, UNCACHED_WORDS) };
        let seed = fresh_seed();
        fill_pattern(heap, seed);
        assert_pattern(heap, seed, "the heap at 250 MHz");
        assert_eq!(in_text(), 0x5053_524D);

        let mut flash = Flash::new(peripherals.FLASH, Config::default()).unwrap();
        assert_flash_works(&mut flash, !seed);

        assert_pattern(heap, seed, "the heap after the flash operations");
        assert_eq!(in_text(), 0x5053_524D);
    }

    #[test]
    fn test_no_flash_page_mapped() {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        let spi0 = esp_hal::peripherals::SPI0::regs();

        for i in 0..property!("mmu.entry_num") {
            spi0.mmu_item_index().write(|w| unsafe { w.bits(i) });
            assert!(
                !spi0.mmu_item_content().read().valid().bit_is_set(),
                "entry {} is mapped",
                i
            );
        }
    }

    // Fails if the clock tree, which light sleep reads, loses the start-up's MPLL rate when the
    // configuration sets none.
    #[cfg(esp32s31)]
    #[test]
    fn test_mpll_rate_without_mpll_config() {
        let mut clocks = ClockConfig::from(esp_hal::clock::CpuClock::default());
        clocks.mpll_clk = None;
        let _peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clocks));

        assert_eq!(
            ClockTree::with(|clocks| clocks.mpll_clk()),
            Some(MpllClkConfig::_400)
        );
        assert_eq!(mpll_clk_frequency(), 400_000_000);
        assert_eq!(mpll_in_hardware(), 400_000_000);
    }

    // The start-up holds a reference to the MPLL. Fails if a request after `init` restarts the
    // MPLL, or if the release stops it.
    #[cfg(esp32s31)]
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

        ClockTree::with(release_mpll_clk);

        assert_eq!(mpll_clk_frequency(), before);
        assert_eq!(mpll_in_hardware(), before);
        assert_eq!(in_text(), 0x5053_524D);
        assert_pattern(heap, seed, "the heap after the MPLL release");
    }

    // Fails if `Psram::new` with another timing changes the MPLL, or if the heap or the code stop
    // working.
    #[cfg(esp32s31)]
    #[test]
    fn test_psram_new_ignores_the_timing() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(peripherals);
        let before = mpll_clk_frequency();
        let timing = PsramTimingParams::MHZ_250;

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
}

/// Flash operations with code in PSRAM.
#[embedded_test::tests(default_timeout = 30)]
mod flash_operations {
    use allocator_api2::boxed::Box;
    use esp_alloc::ExternalMemory;
    use esp_hal::{
        flash::{Config, Error, Flash},
        timer::{PeriodicTimer, systimer::SystemTimer},
    };
    use hil_test::{assert_flash_value_marks, mk_static};

    use super::*;

    /// Asserts that no MMU entry maps flash, as after the start-up. On the ESP32-S31, this MMU maps
    /// only flash.
    fn assert_no_flash_mapped(after: &str) {
        let spi0 = esp_hal::peripherals::SPI0::regs();
        let mapped = (0..property!("mmu.entry_num")).find(|&i| {
            spi0.mmu_item_index().write(|w| unsafe { w.bits(i) });
            spi0.mmu_item_content().read().valid().bit_is_set()
        });
        assert_eq!(mapped, None, "an entry maps flash after {after}");
    }

    // Fails if a page stays mapped, or its cache lines stay: the second `read_encrypted` returns
    // the old words.
    #[test]
    fn test_read_encrypted_sees_new_data() {
        let p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);
        let mut plain = [0u32; 256];
        let mut through_mmu = [0u32; 256];

        flash.read(block, &mut plain).unwrap();
        flash.read_encrypted(block, &mut through_mmu).unwrap();
        assert_no_flash_mapped("the first read_encrypted");
        assert_eq!(plain, through_mmu);

        let data = plain.map(|word| !word);
        // SAFETY: the block is in the booted partition, after the image.
        unsafe {
            flash.erase(block, block + BLOCK).unwrap();
            flash.write(block, &data).unwrap();
        }
        flash.read_encrypted(block, &mut through_mmu).unwrap();
        assert_no_flash_mapped("the second read_encrypted");
        assert_eq!(through_mmu, data);
    }

    // The `flash` test covers `write_encrypted`, which needs its test build.
    #[test]
    fn test_value_marks_the_operation() {
        let p = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let timer = PeriodicTimer::new(SystemTimer::new(p.SYSTIMER).alarm0);
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);
        let (app, _, _) = booted_image(&mut flash, |_, _, _, _| {});
        let buffer = mk_static!([u32; 0x4000], [0; 0x4000]);

        let timer = assert_flash_value_marks(
            timer,
            || flash.read_encrypted(app, buffer).unwrap(),
            "read_encrypted",
        );
        write_sectors(&mut flash, block);
        assert_flash_value_marks(
            timer,
            || erase_sectors(&mut flash, block).unwrap(),
            "erase of 15 sectors",
        );
    }

    // A regression test: `check_buffer` must still refuse a buffer outside DRAM.
    #[test]
    fn test_read_into_psram_is_refused() {
        let p = esp_hal::init(esp_hal::Config::default());
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let mut buffer = Box::new_in([0u32; 4], ExternalMemory);
        assert_eq!(
            flash.read(0x1_0000, &mut buffer[..]),
            Err(Error::NotSupported)
        );
    }
}

/// A stop or a park of the APP core, and a light sleep, during a flash operation.
///
/// In most tests, the APP core erases 15 sectors in one operation. Its timer handler holds it
/// inside the erase until the PRO core releases it, so the stop or the park starts there. No light
/// sleep happens, so the debug probe stays connected.
#[cfg(multi_core)]
#[embedded_test::tests(default_timeout = 30)]
mod stop_and_park_during_erase {
    use core::{ptr::addr_of_mut, sync::atomic::AtomicBool};

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        handler,
        interrupt::InterruptHandler,
        peripherals::{LPWR, Peripherals, TIMG1},
        rtc_cntl::sleep::{LightSleep, LowPower, RtcSleepConfig},
        system::{AppCoreGuard, Cpu, CpuControl, ParkGuard, Stack, is_running},
        time::{Duration, Instant},
        timer::{OneShotTimer, PeriodicTimer, timg::TimerGroup},
    };
    use hil_test::flash_operation;

    use super::*;

    const APP_PERIOD: Duration = Duration::from_micros(200);
    /// The hang bound on one cycle of the APP core's loop.
    const RESUME_BOUND: Duration = Duration::from_secs(4);
    /// The delay from [`arm_pro_timer`] to the PRO core's timer handler.
    const RELEASE_AFTER: Duration = Duration::from_millis(1);
    /// The longest time that the chip needs for a sector erase that the APP core left in progress.
    const SECTOR_ERASE_BOUND: Duration = Duration::from_secs(1);

    const WRITE: u32 = 0;
    const ERASE: u32 = 1;

    static PHASE: AtomicU32 = AtomicU32::new(WRITE);
    /// The erases that the APP core finished, and the read-backs after them.
    static ERASES: AtomicU32 = AtomicU32::new(0);
    static CYCLES: AtomicU32 = AtomicU32::new(0);
    static ERASE_FAILED: AtomicBool = AtomicBool::new(false);
    static READ_BACK_FAILED: AtomicBool = AtomicBool::new(false);
    static HOLD: AtomicBool = AtomicBool::new(false);
    static HELD: AtomicBool = AtomicBool::new(false);
    static RELEASE: AtomicBool = AtomicBool::new(false);
    static RESUMED: AtomicBool = AtomicBool::new(false);
    static APP_RUNS: AtomicU32 = AtomicU32::new(0);
    /// Whether the APP core's timer handler drops [`APP_GUARD`] inside the APP core's erase.
    static STOP_IN_APP_HANDLER: AtomicBool = AtomicBool::new(false);
    /// Whether the PRO core's timer handler ended the park of [`PARKED`].
    static UNPARKED: AtomicBool = AtomicBool::new(false);
    static STOPPED: AtomicBool = AtomicBool::new(false);
    static VALUE_BEFORE_STOP: AtomicU32 = AtomicU32::new(u32::MAX);
    static VALUE_AFTER_STOP: AtomicU32 = AtomicU32::new(u32::MAX);
    static CONDITION_RAN: AtomicBool = AtomicBool::new(false);

    /// The APP core's timer. After the APP core starts it, only the handler uses it.
    static mut APP_TIMER: Option<PeriodicTimer<'static, Blocking>> = None;
    /// The PRO core's timer, used in the same way.
    static mut PRO_TIMER: Option<OneShotTimer<'static, Blocking>> = None;
    /// A park of the APP core that the PRO core's timer handler ends.
    static mut PARKED: Option<ParkGuard<'static>> = None;
    static mut APP_GUARD: Option<AppCoreGuard<'static>> = None;
    /// The stack of the APP core, which a test can start again after a stop.
    static mut APP_STACK: Stack<8192> = Stack::new();

    const APP_OPERATION: u32 = Cpu::AppCpu as u32 + 1;
    const PRO_OPERATION: u32 = Cpu::ProCpu as u32 + 1;

    /// The condition of the tests' light sleeps. The light sleep calls it only after its flash
    /// check passed.
    fn may_sleep() -> bool {
        CONDITION_RAN.store(true, Ordering::Release);
        false
    }

    // Runs on the APP core. The flash value names the APP core for the whole erase, so a handler
    // that finds it in the erase phase interrupted the erase.
    #[handler]
    fn app_timer_handler() {
        // SAFETY: after the APP core starts the timer, only this handler uses `APP_TIMER`.
        let timer = unsafe { (*addr_of_mut!(APP_TIMER)).as_mut() }.unwrap();
        timer.clear_interrupt();
        APP_RUNS.fetch_add(1, Ordering::Release);
        if PHASE.load(Ordering::Acquire) != ERASE || flash_operation() != APP_OPERATION {
            return;
        }
        if STOP_IN_APP_HANDLER.load(Ordering::Acquire) {
            // SAFETY: the test stored the guard first, and does not use it again.
            let guard = unsafe { (*addr_of_mut!(APP_GUARD)).take() };
            // The drop stops the APP core here, and does not return.
            drop(guard);
        }
        if !HOLD.load(Ordering::Acquire) || HELD.load(Ordering::Relaxed) {
            return;
        }
        HELD.store(true, Ordering::Release);
        while !RELEASE.load(Ordering::Acquire) {}
        // A stop or a park that returns without this store did not wait.
        RESUMED.store(true, Ordering::Release);
    }

    fn clear_pro_timer() {
        // SAFETY: after the test arms the timer, only its handler uses `PRO_TIMER`.
        if let Some(timer) = unsafe { (*addr_of_mut!(PRO_TIMER)).as_mut() } {
            timer.clear_interrupt();
        }
    }

    #[handler]
    fn release() {
        clear_pro_timer();
        RELEASE.store(true, Ordering::Release);
        // SAFETY: the test stores the guard before it arms the timer, and does not use it again.
        if let Some(parked) = unsafe { (*addr_of_mut!(PARKED)).take() } {
            UNPARKED.store(true, Ordering::Release);
            drop(parked);
        }
    }

    // Runs inside the PRO core's erase.
    #[handler]
    fn stop_app_core() {
        clear_pro_timer();
        // SAFETY: the test stores the guard before it arms the timer, and does not use it again.
        let guard = unsafe { (*addr_of_mut!(APP_GUARD)).take() };
        VALUE_BEFORE_STOP.store(flash_operation(), Ordering::Relaxed);
        drop(guard);
        VALUE_AFTER_STOP.store(flash_operation(), Ordering::Relaxed);
        STOPPED.store(true, Ordering::Release);
    }

    struct Context {
        flash: Flash<'static, Blocking>,
        timg1: TIMG1<'static>,
        cpu_control: CpuControl<'static>,
        lpwr: LPWR<'static>,
    }

    /// `pro_timer_handler` runs on the PRO core, [`RELEASE_AFTER`] after [`arm_pro_timer`].
    fn setup(pro_timer_handler: InterruptHandler) -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let timg0 = TimerGroup::new(p.TIMG0);
        let mut timer = OneShotTimer::new(timg0.timer0);
        timer.set_interrupt_handler(pro_timer_handler);
        // SAFETY: the timer is not armed yet, so the handler does not use `PRO_TIMER`.
        unsafe { (*addr_of_mut!(PRO_TIMER)).insert(timer) }.listen();
        Context {
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
            timg1: p.TIMG1,
            cpu_control: CpuControl::new(p.CPU_CTRL),
            lpwr: p.LPWR,
        }
    }

    fn arm_pro_timer() {
        // SAFETY: the timer is not armed, so the handler does not use `PRO_TIMER`.
        let timer = unsafe { (*addr_of_mut!(PRO_TIMER)).as_mut() }.unwrap();
        timer.schedule(RELEASE_AFTER).unwrap();
    }

    /// Starts the APP core's timer. It must run on the APP core, so that the handler runs there.
    fn start_app_timer(timg1: TIMG1<'static>) {
        let mut timer = PeriodicTimer::new(TimerGroup::new(timg1).timer0);
        timer.set_interrupt_handler(app_timer_handler);
        // SAFETY: the timer does not run yet, so the handler does not use `APP_TIMER`.
        let timer = unsafe { (*addr_of_mut!(APP_TIMER)).insert(timer) };
        timer.listen();
        timer.start(APP_PERIOD).unwrap();
    }

    fn app_stack() -> &'static mut Stack<8192> {
        // SAFETY: the APP core does not run at any call.
        unsafe { &mut *addr_of_mut!(APP_STACK) }
    }

    /// Starts the APP core, which writes, erases, and reads back the 15 sectors of `block` in a
    /// loop, through the caller's `flash`. The APP core starts its timer if it gets `timg1`.
    fn start_erasing<'a>(
        cpu_control: &mut CpuControl<'static>,
        flash: &'a mut Flash<'static, Blocking>,
        timg1: Option<TIMG1<'static>>,
        block: u32,
    ) -> AppCoreGuard<'a> {
        cpu_control
            .start_app_core(app_stack(), move || {
                PHASE.store(WRITE, Ordering::Release);
                if let Some(timg1) = timg1 {
                    start_app_timer(timg1);
                }
                loop {
                    write_sectors(flash, block);
                    PHASE.store(ERASE, Ordering::Release);
                    if erase_sectors(flash, block).is_err() {
                        ERASE_FAILED.store(true, Ordering::Relaxed);
                    }
                    PHASE.store(WRITE, Ordering::Release);
                    ERASES.fetch_add(1, Ordering::Release);
                    if !sectors_erased(flash, block) {
                        READ_BACK_FAILED.store(true, Ordering::Relaxed);
                    }
                    CYCLES.fetch_add(1, Ordering::Release);
                }
            })
            .unwrap()
    }

    /// Returns once the APP core's timer handler holds the APP core inside an erase.
    fn hold_in_erase() {
        HOLD.store(true, Ordering::Release);
        while !HELD.load(Ordering::Acquire) {
            assert!(
                ERASES.load(Ordering::Acquire) < ATTEMPTS,
                "the APP core's timer never held it inside an erase, in {ATTEMPTS} erases"
            );
        }
    }

    /// Asserts that the APP core's timer handler does not run for `periods` of its periods.
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

    /// Asserts that a stop inside the held erase, after `erases` erases, let the APP core finish
    /// that erase and run no code after it.
    fn assert_stopped_at_the_end_of_the_erase(
        flash: &mut Flash<'static, Blocking>,
        block: u32,
        erases: u32,
    ) {
        assert!(
            RESUMED.load(Ordering::Acquire),
            "the stop returned before the APP core left its timer handler"
        );
        assert_eq!(flash_operation(), 0, "the flash value after the stop");
        assert!(!is_running(Cpu::AppCpu), "the APP core runs after the stop");
        assert_eq!(
            (
                PHASE.load(Ordering::Acquire),
                ERASES.load(Ordering::Acquire)
            ),
            (ERASE, erases),
            "the APP core ran code after its erase"
        );
        assert_app_core_idle(50, "stop");
        assert!(!ERASE_FAILED.load(Ordering::Relaxed), "an erase failed");
        // Before the PRO core erases anything. A stop inside the erase leaves the pattern.
        assert!(
            sectors_erased(flash, block),
            "the stop did not wait for the end of the APP core's erase"
        );
    }

    // Fails if `stop_app_core` does not wait: the flash value stays set, and sectors of the range
    // hold the pattern.
    #[test]
    fn test_stop_waits_for_the_end_of_the_app_core_erase() {
        let mut ctx = setup(release);
        let block = scratch_block(&mut ctx.flash);

        let guard = start_erasing(&mut ctx.cpu_control, &mut ctx.flash, Some(ctx.timg1), block);
        hold_in_erase();
        let erases = ERASES.load(Ordering::Acquire);
        let before = Instant::now();
        arm_pro_timer();
        drop(guard);
        defmt::info!("the stop took {} µs", (Instant::now() - before).as_micros());

        assert_stopped_at_the_end_of_the_erase(&mut ctx.flash, block, erases);
    }

    // A park does not wait for a flash operation.
    #[test]
    fn test_park_lands_inside_the_app_core_erase() {
        let mut ctx = setup(release);
        let block = scratch_block(&mut ctx.flash);
        let mut lpwr = LowPower::new(ctx.lpwr);

        let _running = start_erasing(&mut ctx.cpu_control, &mut ctx.flash, Some(ctx.timg1), block);
        hold_in_erase();
        let cycles = CYCLES.load(Ordering::Acquire);
        arm_pro_timer();
        let parked = ctx.cpu_control.park_core(Cpu::AppCpu);

        assert!(
            RESUMED.load(Ordering::Acquire),
            "the park returned before the APP core left its timer handler"
        );
        assert_eq!(
            flash_operation(),
            APP_OPERATION,
            "the park did not land inside the APP core's erase"
        );
        assert_app_core_idle(50, "park");
        assert_eq!(
            lpwr.sleep_light_if(RtcSleepConfig::default(), may_sleep),
            LightSleep::Refused
        );
        assert!(
            !CONDITION_RAN.load(Ordering::Acquire),
            "a light sleep reached its condition during the APP core's erase"
        );

        drop(parked);
        let resumed = Instant::now();
        while CYCLES.load(Ordering::Acquire) == cycles {
            assert!(
                Instant::now() - resumed < RESUME_BOUND,
                "the APP core did not finish its erase after the park"
            );
        }
        assert!(!ERASE_FAILED.load(Ordering::Relaxed), "the erase failed");
        assert!(
            !READ_BACK_FAILED.load(Ordering::Relaxed),
            "the range was not erased after the park"
        );
    }

    // Fails if `stop_app_core` does not wait: the drop returns before the timer handler ends the
    // park.
    #[test]
    fn test_stop_joins_a_park_inside_the_erase() {
        let mut ctx = setup(release);
        let block = scratch_block(&mut ctx.flash);

        let guard = start_erasing(&mut ctx.cpu_control, &mut ctx.flash, Some(ctx.timg1), block);
        hold_in_erase();
        let erases = ERASES.load(Ordering::Acquire);
        arm_pro_timer();
        let parked = ctx.cpu_control.park_core(Cpu::AppCpu);
        assert_eq!(
            flash_operation(),
            APP_OPERATION,
            "the park did not land inside the APP core's erase"
        );

        // SAFETY: `ctx.cpu_control` lives until the test ends, after the handler drops the guard.
        // The handler runs on the core that parked.
        unsafe {
            *addr_of_mut!(PARKED) = Some(
                core::mem::transmute::<ParkGuard<'_>, ParkGuard<'static>>(parked),
            );
        }
        arm_pro_timer();
        drop(guard);

        assert!(
            UNPARKED.load(Ordering::Acquire),
            "the stop returned while a park held the APP core inside its erase"
        );
        assert_stopped_at_the_end_of_the_erase(&mut ctx.flash, block, erases);
    }

    // As without code in PSRAM: the stop does not wait for the park to end.
    #[test]
    fn test_stop_joins_a_park_outside_an_operation() {
        let mut ctx = setup(release);
        let timg1 = ctx.timg1;

        let guard = ctx
            .cpu_control
            .start_app_core(app_stack(), move || {
                start_app_timer(timg1);
                loop {}
            })
            .unwrap();
        while APP_RUNS.load(Ordering::Acquire) == 0 {}

        let parked = ctx.cpu_control.park_core(Cpu::AppCpu);
        assert_eq!(flash_operation(), 0);
        // A stop that waits for the park never returns, because the park ends after the drop.
        drop(guard);
        assert!(!is_running(Cpu::AppCpu), "the APP core runs after the stop");
        drop(parked);
        assert_app_core_idle(50, "stop");
    }

    // The operation stops unfinished. Fails if `stop_current_core` does not clear the flash value:
    // the value stays set, and the light sleep is refused before its condition.
    #[test]
    fn test_app_core_stops_itself_inside_its_erase() {
        let mut ctx = setup(release);
        let block = scratch_block(&mut ctx.flash);
        let mut lpwr = LowPower::new(ctx.lpwr);

        let guard = start_erasing(&mut ctx.cpu_control, &mut ctx.flash, Some(ctx.timg1), block);
        // SAFETY: the APP core's timer handler drops the guard. The test does not use the guard or
        // `ctx.flash` again.
        unsafe {
            *addr_of_mut!(APP_GUARD) = Some(core::mem::transmute::<
                AppCoreGuard<'_>,
                AppCoreGuard<'static>,
            >(guard));
        }
        STOP_IN_APP_HANDLER.store(true, Ordering::Release);
        let deadline = Instant::now() + RESUME_BOUND;
        while is_running(Cpu::AppCpu) {
            assert!(
                Instant::now() < deadline,
                "the APP core did not stop itself"
            );
        }

        assert_eq!(PHASE.load(Ordering::Acquire), ERASE);
        assert_eq!(flash_operation(), 0, "the stop left the flash value set");
        assert_eq!(
            lpwr.sleep_light_if(RtcSleepConfig::default(), may_sleep),
            LightSleep::Refused
        );
        assert!(
            CONDITION_RAN.load(Ordering::Acquire),
            "a light sleep after the stop was refused before its condition"
        );
        assert_app_core_idle(50, "stop");

        // The next test resets the chip, so the flash chip must first finish the APP core's sector
        // erase.
        let idle = Instant::now() + SECTOR_ERASE_BOUND;
        while Instant::now() < idle {}
    }

    // The flash value names the PRO core, so the stop does not wait for the operation. Fails if
    // the stop waits while the value is not 0: the handler never returns.
    #[test]
    fn test_stop_completes_during_a_pro_core_erase() {
        let mut ctx = setup(stop_app_core);
        let block = scratch_block(&mut ctx.flash);
        let timg1 = ctx.timg1;
        write_sectors(&mut ctx.flash, block);

        let guard = ctx
            .cpu_control
            .start_app_core(app_stack(), move || {
                start_app_timer(timg1);
                loop {}
            })
            .unwrap();
        while APP_RUNS.load(Ordering::Acquire) == 0 {}
        // SAFETY: the PRO core's timer is not armed, so its handler does not use `APP_GUARD`.
        unsafe { *addr_of_mut!(APP_GUARD) = Some(guard) };

        arm_pro_timer();
        erase_sectors(&mut ctx.flash, block).unwrap();

        assert!(
            STOPPED.load(Ordering::Acquire),
            "the timer handler did not run during the erase"
        );
        assert_eq!(
            VALUE_BEFORE_STOP.load(Ordering::Relaxed),
            PRO_OPERATION,
            "the stop did not start inside the PRO core's erase"
        );
        assert_eq!(
            VALUE_AFTER_STOP.load(Ordering::Relaxed),
            PRO_OPERATION,
            "the stop did not return inside the PRO core's erase"
        );
        assert!(!is_running(Cpu::AppCpu), "the APP core runs after the stop");
        assert_app_core_idle(50, "stop");
        assert_eq!(flash_operation(), 0, "the flash value after the erase");
        assert!(
            sectors_erased(&mut ctx.flash, block),
            "the range is not erased"
        );
    }
}

/// A light sleep that a timer handler requests inside a flash erase of its own core.
///
/// `sleep_light_if` calls its condition only after the flash check passed. The condition refuses
/// every light sleep, so the debug probe stays connected.
#[embedded_test::tests(default_timeout = 30)]
mod sleep_during_erase {
    use core::{ptr::addr_of_mut, sync::atomic::AtomicBool};

    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        handler,
        peripherals::{LPWR, Peripherals},
        rtc_cntl::sleep::{LightSleep, LowPower, RtcSleepConfig},
        time::{Duration, Instant},
        timer::{PeriodicTimer, systimer::SystemTimer},
    };

    use super::*;

    const PERIOD: Duration = Duration::from_micros(200);
    /// The requests that show that the handler ran inside the erase.
    const REQUESTS: u32 = 10;

    static REQUESTING: AtomicBool = AtomicBool::new(false);
    /// The handler requests no light sleep from this time on, in microseconds since boot.
    static REQUEST_UNTIL: AtomicU32 = AtomicU32::new(0);
    static REQUESTED: AtomicU32 = AtomicU32::new(0);
    static NOT_REFUSED: AtomicU32 = AtomicU32::new(0);
    static CONDITION_RUNS: AtomicU32 = AtomicU32::new(0);

    /// The timer of the requests. After the test starts it, only the handler uses it.
    static mut TIMER: Option<PeriodicTimer<'static, Blocking>> = None;

    fn now_us() -> u32 {
        Instant::now().duration_since_epoch().as_micros() as u32
    }

    fn may_sleep() -> bool {
        CONDITION_RUNS.fetch_add(1, Ordering::Relaxed);
        false
    }

    #[handler]
    fn sleep_in_erase() {
        // SAFETY: after the test starts the timer, only this handler uses `TIMER`.
        if let Some(timer) = unsafe { (*addr_of_mut!(TIMER)).as_mut() } {
            timer.clear_interrupt();
        }
        let left = REQUEST_UNTIL.load(Ordering::Relaxed).wrapping_sub(now_us()) as i32;
        if !REQUESTING.load(Ordering::Acquire) || left <= 0 {
            return;
        }
        let mut lpwr = LowPower::new(unsafe { LPWR::steal() });
        let result = lpwr.sleep_light_if(RtcSleepConfig::default(), may_sleep);
        REQUESTED.fetch_add(1, Ordering::Relaxed);
        if result != LightSleep::Refused {
            NOT_REFUSED.fetch_add(1, Ordering::Relaxed);
        }
    }

    // The requests cover the first half of an erase of 15 sectors. Fails if the light sleep does
    // not check for a flash operation: every request reaches its condition.
    #[test]
    fn test_sleep_from_a_handler_inside_an_erase_is_refused() {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        let mut flash = Flash::new(p.FLASH, Config::default()).unwrap();
        let block = scratch_block(&mut flash);

        // The length of the erase, so that the requests end well before the erase does.
        write_sectors(&mut flash, block);
        let before = Instant::now();
        erase_sectors(&mut flash, block).unwrap();
        let took = Instant::now() - before;
        write_sectors(&mut flash, block);

        // A SYSTIMER alarm: see `assert_flash_value_marks`.
        let mut timer = PeriodicTimer::new(SystemTimer::new(p.SYSTIMER).alarm0);
        timer.set_interrupt_handler(sleep_in_erase);
        // SAFETY: the timer does not run yet, so the handler does not use `TIMER`.
        let timer = unsafe { (*addr_of_mut!(TIMER)).insert(timer) };
        timer.listen();

        REQUEST_UNTIL.store(
            now_us().wrapping_add((took / 2).as_micros() as u32),
            Ordering::Relaxed,
        );
        REQUESTING.store(true, Ordering::Release);
        // The first request comes one period after the start, inside the erase.
        timer.start(PERIOD).unwrap();
        let erased = erase_sectors(&mut flash, block);
        REQUESTING.store(false, Ordering::Release);
        timer.cancel().unwrap();
        erased.unwrap();

        let requested = REQUESTED.load(Ordering::Relaxed);
        defmt::info!(
            "{} light sleeps requested in an erase of {} µs",
            requested,
            took.as_micros()
        );
        assert!(
            requested >= REQUESTS,
            "the handler requested {requested} light sleeps inside the erase"
        );
        assert_eq!(
            NOT_REFUSED.load(Ordering::Relaxed),
            0,
            "light sleeps inside the erase were not refused"
        );
        assert_eq!(
            CONDITION_RUNS.load(Ordering::Relaxed),
            0,
            "light sleeps reached their condition inside the erase"
        );
        assert!(sectors_erased(&mut flash, block), "the range is not erased");
    }
}

/// Latency during flash operations, with code in PSRAM. Interrupts and threads must wait less than
/// [`BOUND`] microseconds during a write, an erase, and a read of 15 sectors, and while a stop of
/// the APP core waits for its erase.
///
/// The scheduler starts in this module only, because its tick would run under every other test.
#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod latency_during_flash_operations {
    use core::sync::atomic::AtomicBool;

    use allocator_api2::vec::Vec;
    use esp_alloc::ExternalMemory;
    use esp_hal::{
        Blocking,
        flash::{Config, Flash},
        handler,
        interrupt::InterruptHandler,
        peripherals::Peripherals,
        time::{Duration, Instant},
        timer::{PeriodicTimer, systimer::SystemTimer, timg::TimerGroup},
    };
    #[cfg(multi_core)]
    use esp_hal::{
        peripherals::CPU_CTRL,
        system::{Cpu, CpuControl, Stack as CoreStack, is_running},
        timer::systimer::Alarm,
    };
    use esp_rtos::{
        CurrentThreadHandle,
        thread::{Stack, ThreadSpawner},
    };
    #[cfg(multi_core)]
    use hil_test::flash_operation;
    use static_cell::ConstStaticCell;

    use super::*;

    const BOUND: u32 = 1_000;
    const PERIOD: Duration = Duration::from_micros(200);
    /// The samples of each latency that show that it was measured during the operation.
    const SAMPLES_MIN: u32 = 100;
    /// How long the test reads the sectors in a loop, to get [`SAMPLES_MIN`] samples.
    const READ_TIME: Duration = Duration::from_millis(50);

    #[derive(Clone, Copy, Debug, defmt::Format)]
    enum Latency {
        InterruptSameCore,
        ThreadSameCore,
        InterruptOtherCore,
        ThreadOtherCore,
        BareLoopOtherCore,
    }

    static WORST: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
    static SAMPLES: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
    /// The operation, in the low 32 bits of the time since boot in microseconds: from `START`, and
    /// while `OPEN` is set or to `END`.
    static START: AtomicU32 = AtomicU32::new(0);
    static END: AtomicU32 = AtomicU32::new(0);
    static OPEN: AtomicBool = AtomicBool::new(false);
    static THREAD_STACK: ConstStaticCell<Stack<4096>> = ConstStaticCell::new(Stack::new());

    fn now_us() -> u32 {
        Instant::now().duration_since_epoch().as_micros() as u32
    }

    fn during_the_operation(time: u32) -> bool {
        time.wrapping_sub(START.load(Ordering::Acquire)) as i32 >= 0
            && (OPEN.load(Ordering::Acquire)
                || END.load(Ordering::Acquire).wrapping_sub(time) as i32 >= 0)
    }

    fn record(latency: Latency, due: u32, ran: u32) {
        if during_the_operation(due) {
            // An alarm on time after a late one runs less than one period after it.
            let late = (ran.wrapping_sub(due) as i32).max(0) as u32;
            WORST[latency as usize].fetch_max(late, Ordering::Relaxed);
            SAMPLES[latency as usize].fetch_add(1, Ordering::Relaxed);
        }
    }

    /// A periodic timer alarm. Its handler records the time from its last run, less one period, as
    /// the latency of the run. Interrupts masked for a time T give a sample of about T.
    struct GapProbe {
        timer: Mutex<RefCell<Option<PeriodicTimer<'static, Blocking>>>>,
        last: AtomicU32,
        latency: Latency,
    }

    impl GapProbe {
        const fn new(latency: Latency) -> Self {
            Self {
                timer: Mutex::new(RefCell::new(None)),
                last: AtomicU32::new(0),
                latency,
            }
        }

        /// Starts the alarms. The handler runs on the current core.
        fn start(&self, mut timer: PeriodicTimer<'static, Blocking>, handler: InterruptHandler) {
            timer.set_interrupt_handler(handler);
            timer.listen();
            self.last.store(now_us(), Ordering::Relaxed);
            // Stored before it can interrupt, so that the handler can clear the interrupt.
            critical_section::with(|cs| {
                let mut stored = self.timer.borrow_ref_mut(cs);
                stored.insert(timer).start(PERIOD).unwrap();
            });
        }

        fn run(&self) {
            critical_section::with(|cs| {
                if let Some(timer) = self.timer.borrow_ref_mut(cs).as_mut() {
                    timer.clear_interrupt();
                }
            });
            let now = now_us();
            let last = self.last.swap(now, Ordering::Relaxed);
            record(
                self.latency,
                last.wrapping_add(PERIOD.as_micros() as u32),
                now,
            );
        }
    }

    static PROBE_SAME: GapProbe = GapProbe::new(Latency::InterruptSameCore);
    #[cfg(multi_core)]
    static PROBE_OTHER: GapProbe = GapProbe::new(Latency::InterruptOtherCore);

    #[handler]
    fn probe_same_handler() {
        PROBE_SAME.run();
    }

    #[cfg(multi_core)]
    #[handler]
    fn probe_other_handler() {
        PROBE_OTHER.run();
    }

    fn measure_thread(latency: Latency) -> ! {
        loop {
            let due = Instant::now() + PERIOD;
            CurrentThreadHandle::get().delay_until(due);
            record(
                latency,
                due.duration_since_epoch().as_micros() as u32,
                now_us(),
            );
        }
    }

    /// Records the longest gap between two readings of the clock, forever.
    #[cfg(multi_core)]
    fn measure_bare_loop() -> ! {
        let mut last = now_us();
        loop {
            let now = now_us();
            if during_the_operation(now) || during_the_operation(last) {
                WORST[Latency::BareLoopOtherCore as usize]
                    .fetch_max(now.wrapping_sub(last), Ordering::Relaxed);
                SAMPLES[Latency::BareLoopOtherCore as usize].fetch_add(1, Ordering::Relaxed);
            }
            last = now;
        }
    }

    struct Context {
        flash: Flash<'static, Blocking>,
        /// 256 KiB of PSRAM heap with a pattern, so that most of it is read back from PSRAM.
        heap: Vec<u32, ExternalMemory>,
        seed: u32,
        #[cfg(multi_core)]
        cpu_ctrl: CPU_CTRL<'static>,
        #[cfg(multi_core)]
        alarm: Alarm<'static>,
    }

    #[init]
    fn init() -> Context {
        let p: Peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(p);
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);
        let systimer = SystemTimer::new(p.SYSTIMER);
        PROBE_SAME.start(PeriodicTimer::new(systimer.alarm0), probe_same_handler);
        CurrentThreadHandle::get().set_priority(1);
        ThreadSpawner::from_static(THREAD_STACK.take())
            .with_priority(2)
            .with_pinned_to(esp_hal::system::Cpu::ProCpu)
            .spawn(|| measure_thread(Latency::ThreadSameCore))
            .detach();

        let mut heap = Vec::with_capacity_in(UNCACHED_WORDS, ExternalMemory);
        heap.resize(UNCACHED_WORDS, 0);
        let seed = fresh_seed();
        fill_pattern(&mut heap, seed);
        Context {
            flash: Flash::new(p.FLASH, Config::default()).unwrap(),
            heap,
            seed,
            #[cfg(multi_core)]
            cpu_ctrl: p.CPU_CTRL,
            #[cfg(multi_core)]
            alarm: systimer.alarm1,
        }
    }

    /// Asserts that each latency in `measured` stays below [`BOUND`] during `operation`.
    fn assert_latency_during(what: &str, measured: &[Latency], operation: impl FnOnce()) {
        for latency in 0..WORST.len() {
            WORST[latency].store(0, Ordering::Relaxed);
            SAMPLES[latency].store(0, Ordering::Relaxed);
        }
        START.store(now_us(), Ordering::Release);
        OPEN.store(true, Ordering::Release);
        operation();
        END.store(now_us(), Ordering::Release);
        OPEN.store(false, Ordering::Release);
        // A late sample still counts by its due time.
        let settle = Instant::now() + Duration::from_millis(5);
        while Instant::now() < settle {}

        for &latency in measured {
            let worst = WORST[latency as usize].load(Ordering::Relaxed);
            let samples = SAMPLES[latency as usize].load(Ordering::Relaxed);
            defmt::info!("{}, {}: {} µs, {} samples", what, latency, worst, samples);
            assert!(
                samples >= SAMPLES_MIN,
                "{}, {:?}: {} samples",
                what,
                latency,
                samples
            );
            assert!(worst < BOUND, "{}, {:?}: {} µs", what, latency, worst);
        }
    }

    /// Asserts that each latency in `measured` stays below [`BOUND`] during a write, an erase, and
    /// a read of 15 sectors, and that `heap` still holds the pattern for `seed`.
    fn assert_latency_during_flash_operations(
        flash: &mut Flash<'static, Blocking>,
        heap: &[u32],
        seed: u32,
        measured: &[Latency],
    ) {
        let block = scratch_block(flash);
        erase_sectors(flash, block).unwrap();

        assert_latency_during("write", measured, || write_sectors(flash, block));
        assert_latency_during("erase", measured, || erase_sectors(flash, block).unwrap());
        let mut erased = true;
        assert_latency_during("read", measured, || {
            let end = Instant::now() + READ_TIME;
            while Instant::now() < end {
                erased &= sectors_erased(flash, block);
            }
        });

        assert!(erased, "the sectors read back not erased");
        assert_pattern(heap, seed, "the heap after the flash operations");
    }

    #[test]
    fn test_latency_on_this_core_and_a_bare_other_core(mut ctx: Context) {
        #[cfg(multi_core)]
        let _guard = {
            static CORE1_STACK: ConstStaticCell<CoreStack<8192>> =
                ConstStaticCell::new(CoreStack::new());
            static STARTED: AtomicBool = AtomicBool::new(false);
            let alarm = PeriodicTimer::new(ctx.alarm);
            let mut cpu_control = CpuControl::new(ctx.cpu_ctrl);
            let guard = cpu_control
                .start_app_core(CORE1_STACK.take(), move || {
                    PROBE_OTHER.start(alarm, probe_other_handler);
                    STARTED.store(true, Ordering::Release);
                    measure_bare_loop()
                })
                .unwrap();
            while !STARTED.load(Ordering::Acquire) {}
            guard
        };

        assert_latency_during_flash_operations(
            &mut ctx.flash,
            &ctx.heap,
            ctx.seed,
            &[
                Latency::InterruptSameCore,
                Latency::ThreadSameCore,
                #[cfg(multi_core)]
                Latency::InterruptOtherCore,
                #[cfg(multi_core)]
                Latency::BareLoopOtherCore,
            ],
        );
    }

    #[cfg(multi_core)]
    #[test]
    fn test_thread_latency_on_the_other_core(mut ctx: Context) {
        static CORE1_STACK: ConstStaticCell<CoreStack<8192>> =
            ConstStaticCell::new(CoreStack::new());
        static STARTED: AtomicBool = AtomicBool::new(false);
        esp_rtos::start_second_core(ctx.cpu_ctrl, CORE1_STACK.take(), || {
            STARTED.store(true, Ordering::Release);
            measure_thread(Latency::ThreadOtherCore)
        });
        while !STARTED.load(Ordering::Acquire) {}

        assert_latency_during_flash_operations(
            &mut ctx.flash,
            &ctx.heap,
            ctx.seed,
            &[Latency::ThreadOtherCore],
        );
    }

    // The current core must not defer its context switches while the stop waits. Fails if the stop
    // keeps its `SchedulerSuspendGuard` for the wait: the thread does not run. Fails if
    // `stop_app_core` does not wait: the drop returns before enough samples.
    #[cfg(multi_core)]
    #[test]
    fn test_a_thread_runs_while_a_stop_waits(mut ctx: Context) {
        static CORE1_STACK: ConstStaticCell<CoreStack<8192>> =
            ConstStaticCell::new(CoreStack::new());

        let block = scratch_block(&mut ctx.flash);
        write_sectors(&mut ctx.flash, block);

        // esp-rtos runs on the PRO core only, so no thread moves to or from the APP core.
        let mut cpu_control = CpuControl::new(ctx.cpu_ctrl);
        let flash = &mut ctx.flash;
        let guard = cpu_control
            .start_app_core(CORE1_STACK.take(), move || {
                // The stop takes effect before the erase returns.
                let _ = erase_sectors(flash, block);
            })
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(1);
        while flash_operation() != Cpu::AppCpu as u32 + 1 {
            assert!(
                Instant::now() < deadline,
                "the APP core did not start its erase"
            );
        }

        assert_latency_during("stop", &[Latency::ThreadSameCore], || drop(guard));

        assert_eq!(flash_operation(), 0, "the flash value after the stop");
        assert!(!is_running(Cpu::AppCpu), "the APP core runs after the stop");
        assert!(
            sectors_erased(&mut ctx.flash, block),
            "the stop did not wait for the end of the APP core's erase"
        );
    }
}

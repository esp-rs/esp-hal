//! Code in PSRAM: the firmware's code and read-only data run from the PSRAM window.

//% CHIP_FILTER: psram_supports_code_in_psram
//% ENV: ESP_HAL_CONFIG_CODE_IN_PSRAM=true
//% FEATURES: unstable

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

/// Returns the flash offset of a random 64 KiB block after the image in the booted partition. A
/// random block spreads the wear of the tests over the partition.
#[cfg(esp32s31)]
fn scratch_block(flash: &mut esp_hal::flash::Flash<'_, esp_hal::Blocking>) -> u32 {
    const BLOCK: u32 = 0x1_0000;
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
    use core::ptr::addr_of;

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

    fn booted_app_offset(flash: &mut Flash<'_, esp_hal::Blocking>) -> u32 {
        booted_image(flash, |_, _, _, _| {}).0
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
        assert!(start as u32 > in_text as *const () as u32);
        assert!(start as u32 > addr_of!(RODATA) as u32);
        // `Psram::size` is the whole chip.
        assert!(psram.size().is_power_of_two());

        let words = size / 4;
        assert_no_alias(start as *mut u32, words);
        let heap = unsafe { core::slice::from_raw_parts_mut(start as *mut u32, words) };
        // Fails with an access fault if the mapping ends before the range does.
        heap[words - 1] = PATTERN;
        assert_eq!(heap[words - 1], PATTERN);
    }

    // Fails if a flash read changes PSRAM content that the data cache does not hold.
    #[test]
    fn test_heap_survives_flash_read() {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(peripherals);
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

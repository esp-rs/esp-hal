//! Code in PSRAM: the start-up that copies `.text` and `.rodata` from flash into PSRAM.
//!
//! The chip's `riscv_preinit` calls [`start`] after `_start` zeroes `.bss` and before
//! `esp_hal::init`. The flow is the same on every chip. The chip's PSRAM module supplies the
//! parts that differ:
//!
//! - `park_other_core`, which stops the other core before the copy. It is empty on a single-core
//!   chip.
//! - `start_psram`, which starts PSRAM and maps it, and returns its size.
//! - `request_clock`, which gives PSRAM's clock to the clock tree from `esp_hal::init`.
//! - The flash MMU: `FLASH_WINDOW`, `FLASH_VALID`, `FLASH_SENSITIVE`, `FLASH_PADDR`, `flash_entry`,
//!   `set_flash_entry`, `flash_page_shift`, and `flash_entry_maps_psram`.
//! - `IMAGE_CHIP_ID`, the chip ID in the image header.
//! - `efuse_block0`, the address of eFuse block 0, whose register name differs between chips.
//! - `cache_writeback`, `cache_invalidate`, and `cache_invalidate_code`, which invalidates what
//!   fetches the copied code.
//!
//! Where PSRAM has its own MMU and address window (`psram.separate_window`), `.text` and
//! `.rodata` link in that window, and the start-up copies each image segment there. Nothing in
//! `.text` or `.rodata` can run until the copy ends, so `start` and everything it calls are
//! inlined into `riscv_preinit` in `.boot.code_in_psram`. None of it can panic.

use portable_atomic::{AtomicU32, AtomicUsize, Ordering};

use super::implem as chip;

/// The PSRAM size the chip reports, for `Psram::size`.
pub(crate) static SIZE: AtomicUsize = AtomicUsize::new(0);
/// The end of the copied code and read-only data.
pub(crate) static CODE_END: AtomicUsize = AtomicUsize::new(0);
/// The flash offset of the booted app, for `flash_mmu_entry_offset`.
pub(crate) static APP_OFFSET: AtomicU32 = AtomicU32::new(0);

/// The stage the start-up reached, for a probe to read.
///
/// 0: not started. 1: started. 2: starting PSRAM. 3: scanning the flash MMU. 4: copying.
/// 5: unmapping flash. 6: done.
///
/// The start-up stops with:
///
/// - 0x101 (ESP32-S31): the MPLL cannot clock PSRAM at `psram-frequency`.
/// - 0x102 (ESP32-S31): PSRAM reports an unknown size.
/// - 0x301: no flash MMU entry maps the app, or no entry is free.
/// - 0x402: a segment is not word-sized, is out of order or overlaps, or lies past the end of
///   PSRAM.
///
/// If the bootloader started the RTC watchdog, a stop ends in its reset. Otherwise the chip stays
/// stopped.
#[unsafe(no_mangle)]
static ESP_HAL_CODE_IN_PSRAM_STAGE: AtomicU32 = AtomicU32::new(0);

const FLASH_ENTRIES: u32 = property!("mmu.entry_num");
const IMAGE_MAGIC: u32 = 0xE9;

#[inline(always)]
fn stage(n: u32) {
    ESP_HAL_CODE_IN_PSRAM_STAGE.store(n, Ordering::Relaxed);
}

/// Records `n` as the stage and spins.
#[inline(always)]
pub(crate) fn stop(n: u32) -> ! {
    stage(n);
    loop {
        core::hint::spin_loop();
    }
}

// `read_volatile` and `write_volatile` hold a precondition check that can panic in a build with
// debug assertions, so the copy uses the instructions directly.

#[inline(always)]
pub(crate) fn load_word(addr: u32) -> u32 {
    let value: u32;
    unsafe { core::arch::asm!("lw {0}, 0({1})", out(reg) value, in(reg) addr) };
    value
}

#[inline(always)]
pub(crate) fn store_word(addr: u32, value: u32) {
    unsafe { core::arch::asm!("sw {0}, 0({1})", in(reg) value, in(reg) addr) };
}

/// Returns whether flash encryption is on: `SPI_BOOT_CRYPT_CNT` has an odd number of bits set.
///
/// `efuse::flash_encryption` reads through a byte slice with bounds checks, so this function reads
/// the eFuse word itself.
#[inline(always)]
pub(crate) fn flash_encrypted() -> bool {
    let field = crate::efuse::SPI_BOOT_CRYPT_CNT;
    let block0 = chip::efuse_block0();
    let word = load_word(block0.wrapping_add((field.bit_start / 32) * 4));
    let count = (word >> (field.bit_start % 32)) & ((1 << field.bit_count) - 1);
    count.count_ones() % 2 == 1
}

/// The flash MMU, whose page size the bootloader sets from the image header.
struct FlashMmu {
    page_shift: u32,
    sensitive: u32,
}

impl FlashMmu {
    #[inline(always)]
    fn new() -> Self {
        Self {
            page_shift: chip::flash_page_shift(),
            sensitive: if flash_encrypted() {
                chip::FLASH_SENSITIVE
            } else {
                0
            },
        }
    }

    #[inline(always)]
    fn page_size(&self) -> u32 {
        1u32.wrapping_shl(self.page_shift)
    }

    #[inline(always)]
    fn entry_addr(&self, entry: u32) -> u32 {
        chip::FLASH_WINDOW.wrapping_add(entry.wrapping_shl(self.page_shift))
    }

    #[inline(always)]
    fn map(&self, entry: u32, page: u32) {
        chip::set_flash_entry(entry, chip::FLASH_VALID | self.sensitive | page);
        chip::cache_invalidate(self.entry_addr(entry), self.page_size());
    }

    #[inline(always)]
    fn unmap(&self, entry: u32) {
        chip::cache_invalidate(self.entry_addr(entry), self.page_size());
        chip::set_flash_entry(entry, 0);
    }
}

/// Returns whether an entry with `content` maps flash, not PSRAM.
#[inline(always)]
fn maps_flash(content: u32) -> bool {
    content & chip::FLASH_VALID != 0 && !chip::flash_entry_maps_psram(content)
}

/// Reads flash words through one spare flash MMU entry, which it moves as reads cross pages.
struct FlashReader<'a> {
    mmu: &'a FlashMmu,
    entry: u32,
    page: u32,
}

impl FlashReader<'_> {
    #[inline(always)]
    fn read(&mut self, offset: u32) -> u32 {
        let page = offset.wrapping_shr(self.mmu.page_shift);
        if page != self.page {
            self.mmu.map(self.entry, page);
            self.page = page;
        }
        let in_page = offset & self.mmu.page_size().wrapping_sub(1);
        load_word(self.mmu.entry_addr(self.entry).wrapping_add(in_page))
    }
}

/// Starts PSRAM, copies the code into it, and unmaps flash.
#[cfg_attr(psram_separate_window, inline(always))]
pub(crate) fn start() {
    stage(1);
    chip::park_other_core();

    stage(2);
    let size = chip::start_psram();
    SIZE.store(size, Ordering::Relaxed);

    stage(3);
    let mmu = FlashMmu::new();
    let mut spare = None;
    let mut i = FLASH_ENTRIES;
    while i > 0 {
        i = i.wrapping_sub(1);
        if chip::flash_entry(i) & chip::FLASH_VALID == 0 {
            spare = Some(i);
            break;
        }
    }
    let Some(spare) = spare else {
        stop(0x301);
    };
    let mut flash = FlashReader {
        mmu: &mmu,
        entry: spare,
        page: u32::MAX,
    };
    // The bootloader maps the app's first page at a high flash MMU entry. A data page mapped
    // above it does not start with an image header.
    let mut app = None;
    let mut i = FLASH_ENTRIES;
    while i > 0 {
        i = i.wrapping_sub(1);
        let content = chip::flash_entry(i);
        if !maps_flash(content) {
            continue;
        }
        let offset = (content & chip::FLASH_PADDR).wrapping_shl(mmu.page_shift);
        if flash.read(offset) & 0xff == IMAGE_MAGIC
            && flash.read(offset.wrapping_add(12)) & 0xffff == chip::IMAGE_CHIP_ID
        {
            app = Some(offset);
            break;
        }
    }
    let Some(app) = app else {
        stop(0x301);
    };

    stage(4);
    #[cfg(psram_separate_window)]
    copy_segments(&mut flash, app, size as u32);

    stage(5);
    APP_OFFSET.store(app, Ordering::Relaxed);
    // No flash page stays mapped, whatever the bootloader left, so an erase or a write cannot
    // change what the cache holds.
    let mut i = 0;
    while i < FLASH_ENTRIES {
        if maps_flash(chip::flash_entry(i)) {
            mmu.unmap(i);
        }
        i = i.wrapping_add(1);
    }
    stage(6);
}

/// Copies each image segment that is linked in the PSRAM window from flash to its address.
#[cfg(psram_separate_window)]
#[inline(always)]
fn copy_segments(flash: &mut FlashReader<'_>, app: u32, size: u32) {
    const WINDOW_SIZE: u32 = property!("mmu.entry_num") * property!("mmu.page_size");

    let header = flash.read(app);
    let start = super::EXTMEM_ORIGIN as u32;
    let end = start.wrapping_add(size);
    // An ESP-IDF image: a 24-byte header, then segments of load address, length, and data.
    let mut offset = app.wrapping_add(24);
    let mut code_end = start;
    let mut n = 0;
    while n < (header >> 8) & 0xff {
        let load = flash.read(offset);
        let len = flash.read(offset.wrapping_add(4));
        offset = offset.wrapping_add(8);
        if load.wrapping_sub(start) < WINDOW_SIZE {
            if !len.is_multiple_of(4)
                || load < code_end
                || load >= end
                || len > end.wrapping_sub(load)
            {
                stop(0x402);
            }
            let mut done: u32 = 0;
            while done < len {
                let word = flash.read(offset.wrapping_add(done));
                store_word(load.wrapping_add(done), word);
                done = done.wrapping_add(4);
            }
            code_end = load.wrapping_add(len);
        }
        offset = offset.wrapping_add(len);
        n = n.wrapping_add(1);
    }
    let copied = code_end.wrapping_sub(start);
    chip::cache_writeback(start, copied);
    chip::cache_invalidate_code(start, copied);
    CODE_END.store(code_end as usize, Ordering::Relaxed);
}

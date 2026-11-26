//! Code in PSRAM: the start-up that copies `.text` and `.rodata` from flash into PSRAM.
//!
//! `hal_main` calls [`start`], which is in `.init` and runs from flash. `start` calls only inlined
//! code, ROM functions and `.rwtext.checked`: the link fails if `.init` refers to `.text`,
//! `.rodata` or `.rwtext`. The link cannot see a call through a pointer or an address made from
//! constants, so the start-up must use neither.

use portable_atomic::{AtomicU32, AtomicUsize, Ordering};

use super::implem as chip;

/// The PSRAM size the chip reports.
pub(crate) static SIZE: AtomicUsize = AtomicUsize::new(0);
/// The end of the copied code and read-only data.
pub(crate) static CODE_END: AtomicUsize = AtomicUsize::new(0);
/// The flash offset of the booted app, for `flash_mmu_entry_offset`.
pub(crate) static APP_OFFSET: AtomicU32 = AtomicU32::new(0);

const FLASH_ENTRIES: u32 = property!("mmu.entry_num");
const IMAGE_MAGIC: u32 = 0xE9;

/// Why the start-up stopped. The value is the exit code, or the value in `a0`.
///
/// Each chip-specific code has the cfg of the hardware that its check uses.
#[repr(u32)]
pub(crate) enum StopCode {
    /// The MPLL cannot clock PSRAM at 200 MHz.
    #[cfg(soc_has_clock_node_mpll_clk)]
    MpllRate    = 0x101,
    /// PSRAM reports an unknown size.
    #[cfg(psram_octal_spi)]
    UnknownSize = 0x102,
    /// The last flash MMU entry does not map the app, or no entry is free.
    NoMmuEntry  = 0x301,
    /// A segment is not word-sized, is out of order or overlaps, or lies past the end of PSRAM.
    #[cfg(psram_separate_window)]
    BadSegment  = 0x402,
}

/// Stops the start-up with a [`StopCode`].
///
/// With a debugger connected, the program exits through semihosting with `code` as its exit code.
/// Otherwise the function spins with `code` in `a0`. If the bootloader started the RTC watchdog,
/// the spin ends in its reset.
#[inline(always)]
pub(crate) fn stop(code: StopCode) -> ! {
    let code = code as u32;
    if crate::debugger::debugger_connected() {
        // `SYS_EXIT_EXTENDED` with `ADP_Stopped_ApplicationExit`.
        let block = [0x20026, code];
        unsafe {
            core::arch::asm!(
                ".option push",
                ".option norvc",
                "slli x0, x0, 0x1f",
                "ebreak",
                "srai x0, x0, 7",
                ".option pop",
                inout("a0") 0x20 => _,
                in("a1") &block,
            )
        };
    }
    unsafe { core::arch::asm!("2: j 2b", in("a0") code, options(noreturn)) }
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

/// Copies `len` bytes, a multiple of 4, from `src` to `dst`, both mapped and word-aligned.
///
/// The loop moves eight words per pass, then the last words one at a time. It is in assembly so
/// that the words stay in registers.
#[inline(always)]
pub(crate) fn copy_words(src: u32, dst: u32, len: u32) {
    let blocks = len & !31;
    if blocks != 0 {
        unsafe {
            core::arch::asm!(
                "2:",
                "lw {a}, 0({s})",
                "lw {b}, 4({s})",
                "lw {c}, 8({s})",
                "lw {d}, 12({s})",
                "lw {e}, 16({s})",
                "lw {f}, 20({s})",
                "lw {g}, 24({s})",
                "lw {h}, 28({s})",
                "sw {a}, 0({t})",
                "sw {b}, 4({t})",
                "sw {c}, 8({t})",
                "sw {d}, 12({t})",
                "sw {e}, 16({t})",
                "sw {f}, 20({t})",
                "sw {g}, 24({t})",
                "sw {h}, 28({t})",
                "addi {s}, {s}, 32",
                "addi {t}, {t}, 32",
                "bltu {s}, {end}, 2b",
                s = inout(reg) src => _,
                t = inout(reg) dst => _,
                end = in(reg) src.wrapping_add(blocks),
                a = out(reg) _,
                b = out(reg) _,
                c = out(reg) _,
                d = out(reg) _,
                e = out(reg) _,
                f = out(reg) _,
                g = out(reg) _,
                h = out(reg) _,
                options(nostack),
            )
        };
    }
    let mut done = blocks;
    while done < len {
        store_word(dst.wrapping_add(done), load_word(src.wrapping_add(done)));
        done = done.wrapping_add(4);
    }
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

/// Reads flash words through one spare flash MMU entry, which it moves as reads cross pages.
struct FlashReader<'a> {
    mmu: &'a FlashMmu,
    entry: u32,
    page: u32,
}

impl FlashReader<'_> {
    /// Maps the flash page of `offset`, and returns the address of `offset` in the window.
    #[inline(always)]
    fn map(&mut self, offset: u32) -> u32 {
        let page = offset.wrapping_shr(self.mmu.page_shift);
        if page != self.page {
            self.mmu.map(self.entry, page);
            self.page = page;
        }
        let in_page = offset & self.mmu.page_size().wrapping_sub(1);
        self.mmu.entry_addr(self.entry).wrapping_add(in_page)
    }

    #[inline(always)]
    fn read(&mut self, offset: u32) -> u32 {
        load_word(self.map(offset))
    }

    /// Copies `len` bytes, a multiple of 4, from flash at `offset` to `dst`, one flash page at a
    /// time.
    #[cfg(psram_separate_window)]
    #[inline(always)]
    fn copy(&mut self, offset: u32, dst: u32, len: u32) {
        let page_size = self.mmu.page_size();
        let mut done: u32 = 0;
        while done < len {
            let at = offset.wrapping_add(done);
            let src = self.map(at);
            let mut n = page_size.wrapping_sub(at & page_size.wrapping_sub(1));
            if n > len.wrapping_sub(done) {
                n = len.wrapping_sub(done);
            }
            copy_words(src, dst.wrapping_add(done), n);
            done = done.wrapping_add(n);
        }
    }
}

/// Starts PSRAM and copies the code into it.
#[unsafe(link_section = ".init")]
#[inline(never)]
pub(crate) fn start() {
    chip::park_other_core();

    let size = chip::start_psram();
    SIZE.store(size, Ordering::Relaxed);

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
        stop(StopCode::NoMmuEntry);
    };
    let mut flash = FlashReader {
        mmu: &mmu,
        entry: spare,
        page: u32::MAX,
    };
    // The bootloader maps the app's first page at the last flash MMU entry.
    let content = chip::flash_entry(FLASH_ENTRIES.wrapping_sub(1));
    let offset = (content & chip::FLASH_PADDR).wrapping_shl(mmu.page_shift);
    let mut app = None;
    if content & chip::FLASH_VALID != 0
        && flash.read(offset) & 0xff == IMAGE_MAGIC
        && flash.read(offset.wrapping_add(12)) & 0xffff == chip::IMAGE_CHIP_ID
    {
        app = Some(offset);
    }
    let Some(app) = app else {
        stop(StopCode::NoMmuEntry);
    };

    copy_segments(&mut flash, app, size as u32);

    APP_OFFSET.store(app, Ordering::Relaxed);
}

/// Unmaps every flash page, so that an erase or a write cannot leave old data in the cache.
/// `start_main` calls it from PSRAM, because the start-up runs from flash.
#[inline(never)]
pub(crate) fn unmap_flash() {
    let mmu = FlashMmu::new();
    let mut i = 0;
    while i < FLASH_ENTRIES {
        if chip::flash_entry(i) & chip::FLASH_VALID != 0 {
            mmu.unmap(i);
        }
        i = i.wrapping_add(1);
    }
}

/// Copies each image segment that is linked in the PSRAM window from flash to its address.
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
                stop(StopCode::BadSegment);
            }
            flash.copy(offset, load, len);
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

//! Flash MMU access.
//!
//! Everything that has to know which flash is mapped where: dropping the cache
//! lines of a written flash range ([`invalidate_mapped`]), and the temporary
//! mappings that let [`read_flash_encrypted`] read decrypted flash through the
//! cache. Suspending the caches themselves is [`super::cache`]'s job.
//!
//! Nothing here runs with the cache suspended, so these helpers live in flash.
//! The two ESP32 exceptions "writing the MMU table and flushing the cache"
//! are confined to [`esp32_cache`].

use core::{ops::Range, ptr};

use super::Error;

/// Guard for a temporarily mapped flash MMU page.
struct FlashMmapGuard {
    entry_id: u32,
    vaddr: *const u8,
    #[cfg_attr(
        esp32,
        expect(dead_code, reason = "the ESP32 can only flush the whole cache")
    )]
    page_size: u32,
    /// When `false` the entry was already mapped and must not be invalidated.
    owned: bool,
}

impl FlashMmapGuard {
    /// A slot this driver mapped itself, and has to unmap again.
    fn new_owned(entry_id: u32, vaddr: *const u8, page_size: u32) -> Self {
        Self {
            entry_id,
            vaddr,
            page_size,
            owned: true,
        }
    }
}

/// Map a flash physical address to a virtual address for encrypted reading.
///
/// The returned mapping is ready to be read through: a temporary slot can
/// retain cache lines from its previous mapping, so the complete mapping is
/// invalidated before this function returns.
fn map_flash_page(paddr: u32) -> Result<FlashMmapGuard, Error> {
    let page_size = mmu_page_size();
    let page_paddr = paddr & !(page_size - 1);

    if let Some(entry_id) = find_existing_entry(page_paddr) {
        let vaddr = entry_id_to_vaddr(entry_id).ok_or(Error::NotSupported)?;
        invalidate_cache(vaddr as u32, page_size);
        return Ok(FlashMmapGuard {
            entry_id,
            vaddr,
            page_size,
            owned: false,
        });
    }

    // The virtual address is resolved before the entry is written, so a slot
    // without one is never left behind mapped.
    cfg_select! {
        esp32s2 => {
            let entry_id = s2::alloc_entry().ok_or(Error::NotSupported)?;
            let vaddr = s2::map_entry(entry_id, page_paddr)?;
            Ok(FlashMmapGuard::new_owned(entry_id, vaddr, page_size))
        }
        esp32 => {
            let entry_id = find_free_entry().ok_or(Error::NotSupported)?;
            let vaddr = entry_id_to_vaddr(entry_id).ok_or(Error::NotSupported)?;
            // Writing the entry and making it visible to the cache must happen
            // in one cache-off window. The register is looked up before that
            // window: the bounds check of the lookup can call into flash.
            esp32_cache::map_entry(table::entry(entry_id), page_paddr);
            Ok(FlashMmapGuard::new_owned(entry_id, vaddr, page_size))
        }
        _ => {
            let entry_id = find_free_entry().ok_or(Error::NotSupported)?;
            let vaddr = entry_id_to_vaddr(entry_id).ok_or(Error::NotSupported)?;
            write_flash_entry(entry_id, page_paddr);
            invalidate_cache(vaddr as u32, page_size);
            Ok(FlashMmapGuard::new_owned(entry_id, vaddr, page_size))
        }
    }
}

/// Unmap a temporarily mapped flash page and invalidate the cache.
fn unmap_flash_page(guard: FlashMmapGuard) {
    cfg_select! {
        esp32 => {
            esp32_cache::unmap_entry(guard.owned.then(|| table::entry(guard.entry_id)));
        }
        _ => {
            invalidate_cache(guard.vaddr as u32, guard.page_size);
            if guard.owned {
                set_entry_invalid(guard.entry_id);
            }
        }
    }
}

/// Read decrypted flash bytes via temporary MMU mappings.
///
/// The cache must stay on. Callers park the other core and disable interrupts.
pub(super) fn read_flash_encrypted(offset: u32, data: &mut [u32]) -> Result<(), Error> {
    if data.is_empty() {
        return Ok(());
    }

    let bytes = unsafe {
        core::slice::from_raw_parts_mut(
            data.as_mut_ptr().cast::<u8>(),
            data.len() * super::WORD_SIZE as usize,
        )
    };

    let page_size = mmu_page_size();
    let mut remaining = bytes;
    let mut current_offset = offset;

    while !remaining.is_empty() {
        let page_base = current_offset & !(page_size - 1);
        let in_page = (current_offset - page_base) as usize;
        let chunk = remaining.len().min(page_size as usize - in_page);

        let guard = map_flash_page(page_base)?;
        let src = unsafe { guard.vaddr.add(in_page) };
        unsafe {
            ptr::copy_nonoverlapping(src, remaining.as_mut_ptr(), chunk);
        }
        unmap_flash_page(guard);

        current_offset += chunk as u32;
        remaining = &mut remaining[chunk..];
    }

    Ok(())
}

/// Drop the cache lines covering `vaddr..vaddr + size`.
///
/// Both users invalidate flash that may be mapped as code as well as data, so
/// every cache that can hold the line is included.
#[cfg(not(any(esp32, esp32p4, esp32s31)))]
fn invalidate_cache(vaddr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(vaddr, size);
    }
}

#[cfg(any(esp32p4, esp32s31))]
fn invalidate_cache(vaddr: u32, size: u32) {
    const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
    const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    #[cfg(esp32p4)]
    const CACHE_MAP_L2_CACHE: u32 = 1 << 5;

    // IROM and DROM share the same window on the P4, which also needs L2.
    let map = CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | CACHE_MAP_L1_DCACHE;
    #[cfg(esp32p4)]
    let map = map | CACHE_MAP_L2_CACHE;

    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(map, vaddr, size);
    }
}

#[cfg(esp32)]
fn invalidate_cache(_vaddr: u32, _size: u32) {
    // The ESP32 cache cannot invalidate a single address range, so the whole cache is flushed.
    esp32_cache::flush_caches();
}

/// Drop cache lines that map `start..start+len` after the caches are back on.
///
/// IDF invalidates in this order (`spi_flash_check_and_flush_cache` after the
/// operation's cache-restore path). ESP32 cannot invalidate by address and
/// flushes while still off instead.
#[cfg(not(esp32))]
pub(super) fn invalidate_mapped(start: u32, len: u32) {
    if len == 0 {
        return;
    }

    let page_size = mmu_page_size();
    let end = start.saturating_add(len);
    let mut addr = start & !(page_size - 1);

    while addr < end {
        invalidate_mapped_page(addr, page_size);
        addr = addr.saturating_add(page_size);
    }
}

/// Drop cache lines for every MMU entry that maps `page_paddr`.
///
/// One physical page can appear more than once. On C2, C3, and S3 the same
/// entry is also visible through a separate I-bus window, so both vaddrs are
/// invalidated.
#[cfg(not(esp32))]
fn invalidate_mapped_page(page_paddr: u32, page_size: u32) {
    let page = flash_page_number(page_paddr);

    for entry_id in all_entries() {
        if entry_is_valid(entry_id)
            && entry_is_flash_mapping(entry_id)
            && entry_flash_page(entry_id) == page
        {
            invalidate_entry_caches(entry_id, page_size);
        }
    }
}

#[cfg(not(esp32))]
fn invalidate_entry_caches(entry_id: u32, page_size: u32) {
    cfg_select! {
        esp32s2 => {
            if let Some(vaddr) = entry_id_to_vaddr(entry_id) {
                invalidate_cache(vaddr as u32, page_size);
            }
        }
        _ => {
            let offset = entry_id * page_size;
            let drom = memory_range!("DROM").start as u32 + offset;
            let irom = memory_range!("IROM").start as u32 + offset;
            invalidate_cache(drom, page_size);
            if irom != drom {
                invalidate_cache(irom, page_size);
            }
        }
    }
}

/// Virtual address an MMU entry maps to, if it is reachable through a data bus.
fn entry_id_to_vaddr(entry_id: u32) -> Option<*const u8> {
    cfg_select! {
        esp32s2 => s2::entry_id_to_vaddr(entry_id),
        _ => {
            let base = memory_range!("DROM").start as u32;
            Some((base + entry_id * mmu_page_size()) as *const u8)
        }
    }
}

fn find_existing_entry(page_paddr: u32) -> Option<u32> {
    let page = flash_page_number(page_paddr);

    data_entries().find(|&entry_id| {
        entry_is_valid(entry_id)
            && entry_is_flash_mapping(entry_id)
            && entry_flash_page(entry_id) == page
    })
}

fn find_free_entry() -> Option<u32> {
    let entries = data_entries();
    // Skip the last entry reserved for bootloader internal flash access.
    (entries.start..entries.end.saturating_sub(1))
        .rev()
        .find(|&entry_id| !entry_is_valid(entry_id))
}

/// Entries that can back a temporary mapping and be read through as data.
fn data_entries() -> Range<u32> {
    cfg_select! {
        not(soc_has_mmu_table) => 0..property!("mmu.entry_num"),
        esp32s2 => s2::DATA_ENTRY_START..s2::DATA_ENTRY_END,
        // The table peripheral bounds the usable entry ids: `mmu.entry_num`
        // counts logical entries across every bus and indexes out of range on
        // the ESP32, whose table exposes its 64 DROM0 slots only.
        _ => 0..table::entry_count(),
    }
}

/// Every entry that can map flash, including windows that only serve code.
#[cfg(not(esp32))]
fn all_entries() -> Range<u32> {
    cfg_select! {
        // Include the S2 I-bus slots (0..0x80). Skip DBUS2/DPORT (0x140..).
        esp32s2 => 0..s2::DATA_ENTRY_END,
        _ => data_entries(),
    }
}

fn mmu_page_size() -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::mmu_page_size(),
        // The C2 page size is configurable (IDF `mmu_ll_get_page_size`).
        esp32c2 => c2::mmu_page_size(),
        _ => property!("mmu.page_size"),
    }
}

/// Inlined so that the ESP32 can call it from its cache-off window.
#[inline(always)]
fn flash_page_number(page_paddr: u32) -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::flash_page_number(page_paddr),
        esp32c2 => page_paddr >> c2::mmu_page_size().trailing_zeros(),
        _ => page_paddr >> 16,
    }
}

fn entry_is_valid(entry_id: u32) -> bool {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::entry_is_valid(entry_id),
        _ => table::entry_is_valid(entry_id),
    }
}

fn entry_is_flash_mapping(entry_id: u32) -> bool {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::entry_is_flash_mapping(entry_id),
        _ => table::entry_is_flash_mapping(entry_id),
    }
}

fn entry_flash_page(entry_id: u32) -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::entry_flash_page(entry_id),
        _ => table::entry_flash_page(entry_id),
    }
}

/// The ESP32 writes its entries inside a cache-off window instead.
#[cfg(not(any(esp32, esp32s2)))]
fn write_flash_entry(entry_id: u32, page_paddr: u32) {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::write_flash_entry(entry_id, page_paddr),
        _ => table::write_flash_entry(entry_id, page_paddr),
    }
}

/// The ESP32 invalidates its entries inside a cache-off window instead.
#[cfg(not(esp32))]
fn set_entry_invalid(entry_id: u32) {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::set_entry_invalid(entry_id),
        _ => table::set_entry_invalid(entry_id),
    }
}

#[cfg(not(soc_has_mmu_table))]
mod indexed {
    use crate::peripherals::SPI0;

    pub(super) fn mmu_page_size() -> u32 {
        let code = mmu_page_size_code();
        cfg_select! {
            // The S31 flash MMU's maximum page size is 256 KiB.
            esp32s31 => 0x40000 >> code,
            _ => 0x10000 >> code,
        }
    }

    fn mmu_page_size_code() -> u32 {
        let ctrl = SPI0::regs().mmu_power_ctrl().read();
        cfg_select! {
            any(esp32c5, esp32c61, esp32s31) => ctrl.mmu_page_size().bits() as u32,
            _ => ctrl.spi_mmu_page_size().bits() as u32,
        }
    }

    pub(super) fn flash_page_number(page_paddr: u32) -> u32 {
        page_paddr >> mmu_page_size().trailing_zeros()
    }

    fn with_entry<R>(entry_id: u32, f: impl FnOnce() -> R) -> R {
        SPI0::regs()
            .mmu_item_index()
            .write(|w| unsafe { w.mmu_item_index().bits(entry_id) });
        f()
    }

    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        with_entry(entry_id, || {
            SPI0::regs().mmu_item_content().read().valid().bit()
        })
    }

    pub(super) fn entry_is_flash_mapping(entry_id: u32) -> bool {
        cfg_select! {
            any(esp32c5, esp32c61) => with_entry(entry_id, || {
                !SPI0::regs().mmu_item_content().read().access_spiram().bit()
            }),
            _ => {
                let _ = entry_id;
                true
            }
        }
    }

    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        with_entry(entry_id, || {
            SPI0::regs().mmu_item_content().read().paddr().bits() as u32
        })
    }

    pub(super) fn write_flash_entry(entry_id: u32, page_paddr: u32) {
        let page = super::flash_page_number(page_paddr) as u16;
        let encrypted = crate::efuse::flash_encryption();
        with_entry(entry_id, || {
            SPI0::regs().mmu_item_content().write(|w| {
                unsafe { w.paddr().bits(page) };
                #[cfg(any(esp32c5, esp32c61))]
                w.access_spiram().clear_bit();
                w.valid().set_bit();
                if encrypted {
                    w.sensitive().set_bit();
                }
                w
            });
        });
    }

    pub(super) fn set_entry_invalid(entry_id: u32) {
        with_entry(entry_id, || {
            // Match `mmu_ll_set_entry_invalid`: the PAC reset value is not zero on every chip.
            SPI0::regs()
                .mmu_item_content()
                .write(|w| unsafe { w.bits(0) });
        });
    }
}

#[cfg(soc_has_mmu_table)]
mod table {
    use crate::{pac::mmu_table::ENTRY, peripherals::MMU_TABLE};

    /// Look up one MMU entry.
    ///
    /// The ESP32 looks the entry up before it turns the cache off: the bounds
    /// check of the lookup can call into flash, which is unreachable then.
    #[inline(always)]
    pub(super) fn entry(entry_id: u32) -> &'static ENTRY {
        MMU_TABLE::regs().entry(entry_id as usize)
    }

    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        !entry(entry_id).read().invalid().bit()
    }

    pub(super) fn entry_is_flash_mapping(entry_id: u32) -> bool {
        let e = entry(entry_id).read();
        cfg_select! {
            esp32s2 => e.access_flash().bit(),
            esp32s3 => !e.access_spiram().bit(),
            _ => {
                let _ = e;
                true
            }
        }
    }

    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        entry(entry_id).read().paddr().bits() as u32
    }

    // The ESP32 calls the two functions below with the cache off, where a call into flash would
    // hang. Inlining them into the caller in RAM keeps them reachable.
    #[cfg(not(esp32s2))]
    #[inline(always)]
    pub(super) fn write_entry(entry: &ENTRY, page_paddr: u32) {
        let page = super::flash_page_number(page_paddr) as u16;
        entry.write(|w| {
            cfg_select! {
                any(esp32, esp32c2, esp32c3) => unsafe { w.paddr().bits(page as u8) },
                _ => unsafe { w.paddr().bits(page) },
            };
            w.invalid().clear_bit();
            #[cfg(all(soc_has_psram, not(any(esp32, esp32s2))))]
            w.access_spiram().clear_bit();
            w
        });
    }

    #[inline(always)]
    pub(super) fn invalidate_entry(entry: &ENTRY) {
        entry.write(|w| w.invalid().set_bit());
    }

    #[cfg(not(any(esp32, esp32s2)))]
    pub(super) fn write_flash_entry(entry_id: u32, page_paddr: u32) {
        write_entry(entry(entry_id), page_paddr);
    }

    #[cfg(not(esp32))]
    pub(super) fn set_entry_invalid(entry_id: u32) {
        invalidate_entry(entry(entry_id));
    }

    /// The S2 derives its range from the bus layout instead.
    #[cfg(not(esp32s2))]
    pub(super) fn entry_count() -> u32 {
        MMU_TABLE::regs().entry_iter().count() as u32
    }
}

/// Flash MMU and cache maintenance for the ESP32.
///
/// The ESP32 cache must be off while the flash MMU is written or the cache is
/// flushed. Both operations disturb a cache fill that runs at the same time,
/// and the core that waits for the fill then hangs. ESP-IDF turns the cache
/// off around both operations, see `s_do_mapping()` in
/// `components/esp_mm/esp_mmu_map.c`.
///
/// Everything that runs inside the cache-off window must be reachable without
/// flash: the `#[ram]` entry points below only call `#[ram]` guard methods and
/// helpers that are inlined into them.
#[cfg(esp32)]
mod esp32_cache {
    use procmacros::ram;

    use super::table;
    use crate::{flash::cache::CacheGuard, pac::mmu_table::ENTRY};

    /// Map `page_paddr` into `entry` and make the new mapping visible.
    #[ram]
    pub(super) fn map_entry(entry: &ENTRY, page_paddr: u32) {
        let cache = CacheGuard::suspend();
        table::write_entry(entry, page_paddr);
        cache.flush_while_off();
    }

    /// Drop the lines of a temporary mapping and invalidate its entry, if we own it.
    #[ram]
    pub(super) fn unmap_entry(owned_entry: Option<&ENTRY>) {
        let cache = CacheGuard::suspend();
        if let Some(entry) = owned_entry {
            table::invalidate_entry(entry);
        }
        cache.flush_while_off();
    }

    /// Drop every cached line.
    #[ram]
    pub(super) fn flush_caches() {
        let cache = CacheGuard::suspend();
        cache.flush_while_off();
    }
}

#[cfg(esp32c2)]
mod c2 {
    use crate::peripherals::EXTMEM;

    /// 0 = 16 KiB, 1 = 32 KiB, 2 = 64 KiB (`EXTMEM_CACHE_MMU_PAGE_SIZE`).
    pub(super) fn mmu_page_size() -> u32 {
        match EXTMEM::regs()
            .cache_conf_misc()
            .read()
            .cache_mmu_page_size()
            .bits()
        {
            0 => 0x4000,
            1 => 0x8000,
            _ => 0x10000,
        }
    }
}

#[cfg(esp32s2)]
mod s2 {
    use procmacros::ram;

    use super::*;

    pub(super) const DATA_ENTRY_START: u32 = 0x200 / 4;
    pub(super) const DATA_ENTRY_END: u32 = 0x500 / 4;
    const IBUS_ENTRY_END: u32 = 0x300 / 4;

    /// Virtual address for an S2 MMU entry.
    ///
    /// Matches `mmu_ll_entry_id_to_vaddr_base()`: IBUS0/1 at 0x4000_0000,
    /// DROM/DBUS at 0x3F00_0000. DBUS2/DPORT (0x140..) is 32-bit access only
    /// and has no usable base here.
    pub(super) fn entry_id_to_vaddr(entry_id: u32) -> Option<*const u8> {
        let page_size = mmu_page_size();
        let (base, relative) = match entry_id {
            0x00..0x40 => (0x4000_0000u32, entry_id),
            0x40..0x80 => (0x4000_0000u32, entry_id - 0x40),
            0x80..0xC0 => (0x3F00_0000u32, entry_id - 0x80),
            0xC0..0x100 => (0x3F00_0000u32, entry_id - 0xC0),
            0x100..0x140 => (0x3F00_0000u32, entry_id - 0x100),
            _ => return None,
        };
        Some((base + relative * page_size) as *const u8)
    }

    /// Pick a free MMU slot, preferring IBUS2/DROM entries first.
    ///
    /// Only data-bus entries are considered: on ESP32-S2, DROM (entries
    /// 128..192) is backed by the I-cache bus (`drom0_in_icache = 1` in
    /// ESP-IDF) but is addressed through 0x3F00_0000, while IBUS0/1 is not
    /// reachable with 8-bit loads.
    pub(super) fn alloc_entry() -> Option<u32> {
        (DATA_ENTRY_START..IBUS_ENTRY_END - 1)
            .rev()
            .find(|&entry_id| !entry_is_valid(entry_id))
            .or_else(find_free_entry)
    }

    fn entry_uses_ibus(entry_id: u32) -> bool {
        (DATA_ENTRY_START..IBUS_ENTRY_END).contains(&entry_id)
    }

    #[ram]
    fn mmu_rom_set(entry_id: u32, access: u32, vaddr: u32, paddr: u32) -> i32 {
        unsafe extern "C" {
            fn Cache_Ibus_MMU_Set(
                ext_ram: u32,
                vaddr: u32,
                paddr: u32,
                psize: u32,
                num: u32,
                fixed: u32,
            ) -> i32;
            fn Cache_Dbus_MMU_Set(
                ext_ram: u32,
                vaddr: u32,
                paddr: u32,
                psize: u32,
                num: u32,
                fixed: u32,
            ) -> i32;
        }

        unsafe {
            if entry_uses_ibus(entry_id) {
                Cache_Ibus_MMU_Set(access, vaddr, paddr, 64, 1, 0)
            } else {
                Cache_Dbus_MMU_Set(access, vaddr, paddr, 64, 1, 0)
            }
        }
    }

    /// Map an MMU entry to a flash page via ROM (must run from IRAM on ESP32-S2).
    #[ram]
    pub(super) fn map_entry(entry_id: u32, page_paddr: u32) -> Result<*const u8, Error> {
        let vaddr = entry_id_to_vaddr(entry_id).ok_or(Error::NotSupported)?;
        let page_size = mmu_page_size();
        let vaddr_u32 = vaddr as u32;
        invalidate_cache(vaddr_u32, page_size);
        if mmu_rom_set(entry_id, 1 << 15, vaddr_u32, page_paddr) != 0 {
            return Err(Error::NotSupported);
        }
        invalidate_cache(vaddr_u32, page_size);
        Ok(vaddr)
    }
}

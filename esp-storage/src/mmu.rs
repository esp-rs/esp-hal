//! Temporary MMU mapping for reading encrypted flash through the cache.
//!
//! Maps a flash physical page into an unused MMU entry, reads decrypted data via
//! the cache, then unmaps and invalidates the cache line.

use core::ptr;

#[cfg(esp32)]
use esp32 as pac;
#[cfg(esp32c2)]
use esp32c2 as pac;
#[cfg(esp32c3)]
use esp32c3 as pac;
#[cfg(esp32c5)]
use esp32c5 as pac;
#[cfg(esp32c6)]
use esp32c6 as pac;
#[cfg(esp32c61)]
use esp32c61 as pac;
#[cfg(esp32h2)]
use esp32h2 as pac;
#[cfg(esp32p4)]
use esp32p4 as pac;
#[cfg(esp32s2)]
use esp32s2 as pac;
#[cfg(esp32s3)]
use esp32s3 as pac;
#[cfg(esp32s31)]
use esp32s31 as pac;
#[cfg(not(soc_has_mmu_table))]
use indexed::*;
#[cfg(soc_has_mmu_table)]
use table::*;

use crate::FlashStorageError;

/// Guard for a temporarily mapped flash MMU page.
pub(crate) struct FlashMmapGuard {
    entry_id: u32,
    vaddr: *const u8,
    #[cfg_attr(
        esp32,
        expect(dead_code, reason = "the ESP32 can only flush the whole cache")
    )]
    page_size: u32,
    /// When `false` the entry was already mapped and must not be invalidated
    owned: bool,
}

impl FlashMmapGuard {
    pub(crate) fn vaddr(&self) -> *const u8 {
        self.vaddr
    }

    fn new(entry_id: u32, page_size: u32, owned: bool) -> Result<Self, FlashStorageError> {
        let vaddr = entry_id_to_vaddr(entry_id);
        if vaddr.is_null() {
            if owned {
                set_entry_invalid(entry_id);
            }
            return Err(FlashStorageError::NotSupported);
        }
        Ok(Self {
            entry_id,
            vaddr,
            page_size,
            owned,
        })
    }
}

/// Map a flash physical address to a virtual address for encrypted reading.
///
/// The returned mapping is ready to be read through: a temporary slot can retain cache lines from
/// its previous mapping, so the complete mapping is invalidated before this function returns.
pub(crate) fn map_flash_page(paddr: u32) -> Result<FlashMmapGuard, FlashStorageError> {
    let page_size = mmu_page_size();
    let page_paddr = paddr & !(page_size - 1);

    if let Some(entry_id) = find_existing_entry(page_paddr) {
        let guard = FlashMmapGuard::new(entry_id, page_size, false)?;
        invalidate_cache(guard.vaddr as u32, page_size);
        return Ok(guard);
    }

    cfg_select! {
        esp32s2 => {
            let entry_id = s2::alloc_entry();
            s2::map_entry(entry_id, page_paddr)?;
            FlashMmapGuard::new(entry_id, page_size, true)
        }
        esp32 => {
            let entry_id = find_free_entry().ok_or(FlashStorageError::NotSupported)?;
            let guard = FlashMmapGuard::new(entry_id, page_size, true)?;
            // Writing the entry and making it visible to the cache must happen in one
            // cache-off window.
            esp32_cache::map_entry(entry(entry_id), page_paddr);
            Ok(guard)
        }
        _ => {
            let entry_id = find_free_entry().ok_or(FlashStorageError::NotSupported)?;
            write_flash_entry(entry_id, page_paddr);
            let guard = FlashMmapGuard::new(entry_id, page_size, true)?;
            invalidate_cache(guard.vaddr as u32, page_size);
            Ok(guard)
        }
    }
}

/// Unmap a temporarily mapped flash page and invalidate the cache.
pub(crate) fn unmap_flash_page(guard: FlashMmapGuard) {
    #[cfg(esp32s2)]
    if guard.owned {
        s2::unmap_entry(guard.entry_id, guard.vaddr as u32, guard.page_size);
        return;
    }

    cfg_select! {
        esp32 => {
            esp32_cache::unmap_entry(guard.owned.then(|| entry(guard.entry_id)));
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
pub(crate) fn read_flash_encrypted(offset: u32, bytes: &mut [u8]) -> Result<(), FlashStorageError> {
    if bytes.is_empty() {
        return Ok(());
    }

    let page_size = mmu_page_size();
    let mut remaining = bytes;
    let mut current_offset = offset;

    while !remaining.is_empty() {
        let page_base = current_offset & !(page_size - 1);
        let in_page = (current_offset - page_base) as usize;
        let chunk = remaining.len().min(page_size as usize - in_page);

        crate::maybe_with_critical_section(|| {
            let guard = map_flash_page(page_base)?;
            let src = unsafe { guard.vaddr().add(in_page) };
            unsafe {
                ptr::copy_nonoverlapping(src, remaining.as_mut_ptr(), chunk);
            }
            unmap_flash_page(guard);
            Ok(())
        })?;

        current_offset += chunk as u32;
        remaining = &mut remaining[chunk..];
    }

    Ok(())
}

/// Invalidate cache lines covering a flash physical address range.
pub(crate) fn invalidate_flash_cache(start: u32, len: u32) {
    if len == 0 {
        return;
    }

    let page_size = mmu_page_size();
    let page_start = start & !(page_size - 1);
    let end = start.saturating_add(len);
    let mut addr = page_start;

    while addr < end {
        if let Some(entry_id) = find_existing_entry(addr) {
            let vaddr = entry_id_to_vaddr(entry_id);
            if !vaddr.is_null() {
                invalidate_cache(vaddr as u32, page_size);
            }
        }
        addr = addr.saturating_add(page_size);
    }
}

#[cfg(not(any(esp32, esp32p4, esp32s31)))]
fn invalidate_cache(vaddr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(vaddr, size);
    }
}

#[cfg(esp32s31)]
fn invalidate_cache(vaddr: u32, size: u32) {
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;

    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, vaddr, size);
    }
}

#[cfg(esp32p4)]
fn invalidate_cache(vaddr: u32, size: u32) {
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    const CACHE_MAP_L2_CACHE: u32 = 1 << 5;

    unsafe extern "C" {
        fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE | CACHE_MAP_L2_CACHE, vaddr, size);
    }
}

#[cfg(esp32)]
fn invalidate_cache(_vaddr: u32, _size: u32) {
    // The ESP32 cache cannot invalidate a single address range, so the whole cache is flushed.
    esp32_cache::flush_caches();
}

#[cfg(not(esp32s2))]
const FLASH_VADDR_BASE: u32 = memory_range!("DROM").start as u32;

fn entry_id_to_vaddr(entry_id: u32) -> *const u8 {
    cfg_select! {
        esp32s2 => s2::entry_id_to_vaddr(entry_id),
        _ => (FLASH_VADDR_BASE + entry_id * mmu_page_size()) as *const u8,
    }
}

fn find_existing_entry(page_paddr: u32) -> Option<u32> {
    let page = flash_page_number(page_paddr);
    let (start, end) = mmu_entry_scan_range();

    (start..end).find(|&entry_id| {
        entry_is_valid(entry_id)
            && entry_is_flash_mapping(entry_id)
            && entry_flash_page(entry_id) == page
    })
}

fn find_free_entry() -> Option<u32> {
    let (start, end) = mmu_entry_scan_range();
    // Skip the last entry — reserved for bootloader internal flash access.
    (start..end.saturating_sub(1))
        .rev()
        .find(|&entry_id| !entry_is_valid(entry_id))
}

fn mmu_entry_scan_range() -> (u32, u32) {
    cfg_select! {
        not(soc_has_mmu_table) => (0, indexed::entry_count()),
        esp32s2 => (s2::DATA_ENTRY_START, s2::DATA_ENTRY_END),
        _ => (0, table::entry_count()),
    }
}

#[cfg(not(soc_has_mmu_table))]
mod indexed {
    use super::*;

    #[inline(always)]
    fn spi0() -> &'static pac::spi0::RegisterBlock {
        unsafe { &*pac::SPI0::ptr() }
    }

    pub(super) fn mmu_page_size() -> u32 {
        let code = mmu_page_size_code();
        cfg_select! {
            // The S31 flash MMU's maximum page size is 256 KiB.
            esp32s31 => 0x40000 >> code,
            // Other indexed flash MMUs have a 64 KiB maximum page size.
            _ => 0x10000 >> code,
        }
    }

    fn mmu_page_size_code() -> u32 {
        let ctrl = spi0().mmu_power_ctrl().read();
        cfg_select! {
            any(esp32c5, esp32c61, esp32s31) => ctrl.mmu_page_size().bits() as u32,
            _ => ctrl.spi_mmu_page_size().bits() as u32,
        }
    }

    fn page_size_shift(page_size: u32) -> u32 {
        match page_size {
            0x40000 => 18,
            0x20000 => 17,
            0x10000 => 16,
            0x8000 => 15,
            0x4000 => 14,
            0x2000 => 13,
            _ => 16,
        }
    }

    pub(super) fn flash_page_number(page_paddr: u32) -> u32 {
        page_paddr >> page_size_shift(mmu_page_size())
    }

    pub(super) fn entry_is_flash_mapping(entry_id: u32) -> bool {
        with_entry(entry_id, flash_mapping_from_entry)
    }

    fn flash_mapping_from_entry(entry: pac::spi0::mmu_item_content::R) -> bool {
        #[cfg(any(esp32c5, esp32c61))]
        {
            !entry.access_spiram().bit()
        }
        #[cfg(not(any(esp32c5, esp32c61)))]
        {
            let _ = entry;
            true
        }
    }

    fn with_entry<R>(entry_id: u32, f: impl FnOnce(pac::spi0::mmu_item_content::R) -> R) -> R {
        select_entry(entry_id);
        f(spi0().mmu_item_content().read())
    }

    fn select_entry(entry_id: u32) {
        spi0()
            .mmu_item_index()
            .write(|w| unsafe { w.mmu_item_index().bits(entry_id) });
    }

    pub(super) fn write_flash_entry(entry_id: u32, page_paddr: u32) {
        let page = flash_page_number(page_paddr) as u16;
        let encrypted = esp_hal::efuse::flash_encryption();
        select_entry(entry_id);
        spi0().mmu_item_content().write(|w| {
            unsafe { w.paddr().bits(page) };
            #[cfg(any(esp32c5, esp32c61))]
            w.access_spiram().clear_bit();
            w.valid().set_bit();
            if encrypted {
                w.sensitive().set_bit()
            } else {
                w
            }
        });
    }

    pub(super) fn set_entry_invalid(entry_id: u32) {
        select_entry(entry_id);
        // Match `mmu_ll_set_entry_invalid`: the PAC reset value is not zero on every chip.
        spi0().mmu_item_content().write(|w| unsafe { w.bits(0) });
    }

    pub(super) fn entry_count() -> u32 {
        property!("mmu.entry_num")
    }

    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        with_entry(entry_id, |entry| entry.valid().bit())
    }

    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        with_entry(entry_id, |entry| entry.paddr().bits() as u32)
    }
}

#[cfg(soc_has_mmu_table)]
mod table {
    use super::*;

    #[inline(always)]
    fn mmu_table() -> &'static pac::mmu_table::RegisterBlock {
        unsafe { &*pac::MMU_TABLE::ptr() }
    }

    #[inline(always)]
    pub(super) fn mmu_page_size() -> u32 {
        property!("mmu.page_size")
    }

    #[inline(always)]
    pub(super) fn flash_page_number(page_paddr: u32) -> u32 {
        page_paddr >> 16
    }

    pub(super) fn entry_is_flash_mapping(entry_id: u32) -> bool {
        with_entry(entry_id, flash_mapping_from_entry)
    }

    fn flash_mapping_from_entry(entry: pac::mmu_table::entry::R) -> bool {
        #[cfg(esp32s2)]
        {
            entry.access_flash().bit()
        }
        #[cfg(esp32s3)]
        {
            !entry.access_spiram().bit()
        }
        #[cfg(not(any(esp32s2, esp32s3)))]
        {
            let _ = entry;
            true
        }
    }

    fn with_entry<R>(entry_id: u32, f: impl FnOnce(pac::mmu_table::entry::R) -> R) -> R {
        f(entry(entry_id).read())
    }

    /// Look up one MMU entry.
    ///
    /// The ESP32 looks the entry up before it turns the cache off: the bounds check of the
    /// lookup can call into flash, which is not reachable with the cache off.
    #[inline(always)]
    pub(super) fn entry(entry_id: u32) -> &'static pac::mmu_table::ENTRY {
        mmu_table().entry(entry_id as usize)
    }

    // The ESP32 calls the two functions below with the cache off, where a call into flash would
    // hang. Inlining them into the caller in RAM keeps them reachable.
    #[cfg(not(esp32s2))]
    #[inline(always)]
    pub(super) fn write_entry(entry: &pac::mmu_table::ENTRY, page_paddr: u32) {
        let page = flash_page_number(page_paddr) as u16;
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
    pub(super) fn invalidate_entry(entry: &pac::mmu_table::ENTRY) {
        entry.write(|w| w.invalid().set_bit());
    }

    #[inline(always)]
    pub(super) fn set_entry_invalid(entry_id: u32) {
        invalidate_entry(entry(entry_id));
    }

    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        with_entry(entry_id, |entry| !entry.invalid().bit())
    }

    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        with_entry(entry_id, |entry| entry.paddr().bits() as u32)
    }

    // The ESP32 writes the entry with the cache off, and so uses `write_entry` directly.
    #[cfg(not(any(esp32, esp32s2)))]
    #[inline(always)]
    pub(super) fn write_flash_entry(entry_id: u32, page_paddr: u32) {
        write_entry(entry(entry_id), page_paddr);
    }

    #[cfg(not(esp32s2))]
    pub(super) fn entry_count() -> u32 {
        esp_hal::peripherals::MMU_TABLE::regs().entry_iter().count() as u32
    }
}

/// Flash MMU and cache maintenance for the ESP32.
///
/// The ESP32 cache must be off while the flash MMU is written or the cache is flushed. Both
/// operations disturb a cache fill that runs at the same time, and the core that waits for the
/// fill then hangs. ESP-IDF turns the cache off around both operations, see `s_do_mapping()` in
/// `components/esp_mm/esp_mmu_map.c`.
///
/// Code that runs with the cache off must not access flash, which is why the functions below are
/// placed in RAM and only call ROM code.
#[cfg(esp32)]
mod esp32_cache {
    use procmacros::ram;

    use super::*;

    unsafe extern "C" {
        fn Cache_Flush_rom(cpu: u32);
    }

    /// The caches that were on before this crate turned them off.
    #[derive(Clone, Copy)]
    struct CachesOn {
        pro: bool,
        app: bool,
    }

    #[inline(always)]
    fn dport() -> &'static pac::dport::RegisterBlock {
        unsafe { &*pac::DPORT::ptr() }
    }

    /// Turn off the cache of both cores.
    ///
    /// The flash MMU is shared, so the other core must not read through it either. The caller
    /// makes sure that the other core does not execute from flash in the meantime.
    #[inline(always)]
    fn cache_off() -> CachesOn {
        let on = CachesOn {
            pro: dport().pro_cache_ctrl().read().pro_cache_enable().bit(),
            app: dport().app_cache_ctrl().read().app_cache_enable().bit(),
        };

        // A cache must be idle before it is turned off, see `cache_ll_l1_disable_cache()`.
        if on.pro {
            while dport().pro_dcache_dbug0().read().pro_cache_state().bits() != 1 {}
            dport()
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_cache_enable().clear_bit());
        }
        if on.app {
            while dport().app_dcache_dbug0().read().app_cache_state().bits() != 1 {}
            dport()
                .app_cache_ctrl()
                .modify(|_, w| w.app_cache_enable().clear_bit());
        }

        // Complete the writes above before the flash MMU is written.
        let _ = dport().pro_cache_ctrl().read();

        on
    }

    /// Drop every cached line, so that both cores see the current mapping.
    #[inline(always)]
    fn flush(on: CachesOn) {
        // A cache that is off holds no lines that its core can hit.
        unsafe {
            if on.pro {
                Cache_Flush_rom(0);
            }
            if on.app {
                Cache_Flush_rom(1);
            }
        }
    }

    #[inline(always)]
    fn cache_on(on: CachesOn) {
        if on.pro {
            dport()
                .pro_cache_ctrl()
                .modify(|_, w| w.pro_cache_enable().set_bit());
        }
        if on.app {
            dport()
                .app_cache_ctrl()
                .modify(|_, w| w.app_cache_enable().set_bit());
        }

        // The caller returns to flash right away, so make sure that the cache is on again before
        // the next instruction is fetched: reading the register back completes the write.
        let _ = dport().pro_cache_ctrl().read();
    }

    /// Map `page_paddr` into `entry` and make the new mapping visible.
    #[ram]
    pub(super) fn map_entry(entry: &pac::mmu_table::ENTRY, page_paddr: u32) {
        let on = cache_off();
        write_entry(entry, page_paddr);
        flush(on);
        cache_on(on);
    }

    /// Drop the lines of a temporary mapping and invalidate its entry, if we own it.
    #[ram]
    pub(super) fn unmap_entry(owned_entry: Option<&pac::mmu_table::ENTRY>) {
        let on = cache_off();
        if let Some(entry) = owned_entry {
            invalidate_entry(entry);
        }
        flush(on);
        cache_on(on);
    }

    /// Drop every cached line.
    #[ram]
    pub(super) fn flush_caches() {
        let on = cache_off();
        flush(on);
        cache_on(on);
    }
}

#[cfg(esp32s2)]
mod s2 {
    use procmacros::ram;

    use super::*;

    pub(super) const DATA_ENTRY_START: u32 = 0x200 / 4;
    pub(super) const DATA_ENTRY_END: u32 = 0x500 / 4;
    const IBUS_ENTRY_END: u32 = 0x300 / 4;
    /// Dedicated scratch slot on the DROM / IBUS2 bus (entry 191 reserved for the bootloader).
    const TEMP_ENTRY_ID: u32 = IBUS_ENTRY_END - 2;

    /// Virtual address for an S2 MMU entry (DROM/IBUS2 or DBUS0/DBUS1).
    ///
    /// On ESP32-S2, DROM (entries 128..192) is backed by the I-cache bus
    /// (`drom0_in_icache = 1` in ESP-IDF). DBUS2/DPORT must not be used here.
    pub(super) fn entry_id_to_vaddr(entry_id: u32) -> *const u8 {
        let page_size = mmu_page_size();
        // Matches `mmu_ll_entry_id_to_vaddr_base()` for data-bus entries.
        // Do not use IBUS0/1/2 (entries 0..128): those are not reachable with 8-bit
        // loads. Do not use DBUS2/DPORT (entries 320..384): 32-bit access only.
        let relative = match entry_id {
            0x80..0xC0 => entry_id - 0x80,
            0xC0..0x100 => entry_id - 0xC0,
            0x100..0x140 => entry_id - 0x100,
            _ => return ptr::null(),
        };
        (0x3F00_0000u32 + relative * page_size) as *const u8
    }

    /// Pick a free MMU slot, preferring IBUS2/DROM entries first.
    pub(super) fn alloc_entry() -> u32 {
        for entry_id in (DATA_ENTRY_START..IBUS_ENTRY_END.saturating_sub(1)).rev() {
            if !entry_is_valid(entry_id) {
                return entry_id;
            }
        }
        find_free_entry().unwrap_or(TEMP_ENTRY_ID)
    }

    /// Returns `true` for DROM MMU slots (128..192) which live on the I-cache bus on ESP32-S2.
    fn entry_uses_ibus(entry_id: u32) -> bool {
        (DATA_ENTRY_START..IBUS_ENTRY_END).contains(&entry_id)
    }

    /// Program one MMU slot through the correct ROM helper (I-bus vs D-bus).
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
    pub(super) fn map_entry(
        entry_id: u32,
        page_paddr: u32,
    ) -> Result<*const u8, FlashStorageError> {
        let vaddr = entry_id_to_vaddr(entry_id);
        if vaddr.is_null() {
            return Err(FlashStorageError::NotSupported);
        }
        let page_size = mmu_page_size();
        let vaddr_u32 = vaddr as u32;
        invalidate_cache(vaddr_u32, page_size);
        let rc = mmu_rom_set(entry_id, 1 << 15, vaddr_u32, page_paddr);
        if rc != 0 {
            return Err(FlashStorageError::NotSupported);
        }
        invalidate_cache(vaddr_u32, page_size);
        Ok(vaddr)
    }

    /// Drop cached lines for a temporary mapping. The MMU slot is left mapped so the next
    /// remap can overwrite it without touching unrelated I/D-cache state.
    #[ram]
    pub(super) fn unmap_entry(_entry_id: u32, vaddr: u32, page_size: u32) {
        invalidate_cache(vaddr, page_size);
    }
}

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
#[cfg(not(soc_has_mmu_table))]
use indexed::*;
#[cfg(soc_has_mmu_table)]
use table::*;

use crate::FlashStorageError;

/// Guard for a temporarily mapped flash MMU page.
pub(crate) struct FlashMmapGuard {
    entry_id: u32,
    vaddr: *const u8,
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
pub(crate) fn map_flash_page(paddr: u32) -> Result<FlashMmapGuard, FlashStorageError> {
    let page_size = mmu_page_size();
    let page_paddr = paddr & !(page_size - 1);

    if let Some(entry_id) = find_existing_entry(page_paddr) {
        return FlashMmapGuard::new(entry_id, page_size, false);
    }

    cfg_select! {
        esp32s2 => {
            let entry_id = s2::alloc_entry();
            s2::map_entry(entry_id, page_paddr)?;
            FlashMmapGuard::new(entry_id, page_size, true)
        },
        _ => {
            let entry_id = find_free_entry().ok_or(FlashStorageError::NotSupported)?;
            write_flash_entry(entry_id, page_paddr);
            FlashMmapGuard::new(entry_id, page_size, true)
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

    invalidate_cache(guard.vaddr as u32, guard.page_size);
    if guard.owned {
        set_entry_invalid(guard.entry_id);
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
            invalidate_cache(guard.vaddr() as u32, chunk as u32);
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

#[cfg(not(any(esp32, esp32p4)))]
fn invalidate_cache(vaddr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(vaddr, size);
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
    unsafe extern "C" {
        fn Cache_Flush_rom(cpu: u32);
    }
    unsafe {
        Cache_Flush_rom(0);
        Cache_Flush_rom(1);
    }
}

#[cfg(not(esp32s2))]
const FLASH_VADDR_BASE: u32 = cfg_select! {
    any(esp32c5, esp32c6, esp32c61, esp32h2) => 0x4200_0000,
    esp32p4 => 0x4000_0000,
    any(esp32c3, esp32c2, esp32s3) => 0x3C00_0000,
    _ => 0x3F40_0000,
};

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
        _ => (0, table::ENTRY_COUNT),
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
        match mmu_page_size_code() {
            0 => 0x10000,
            1 => 0x8000,
            2 => 0x4000,
            _ => 0x2000,
        }
    }

    fn mmu_page_size_code() -> u32 {
        let ctrl = spi0().mmu_power_ctrl().read();
        cfg_select! {
            any(esp32c5, esp32c61) => ctrl.mmu_page_size().bits() as u32,
            _ => ctrl.spi_mmu_page_size().bits() as u32,
        }
    }

    fn page_size_shift(page_size: u32) -> u32 {
        match page_size {
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
        spi0().mmu_item_content().reset();
    }

    pub(super) fn entry_count() -> u32 {
        cfg_select! {
            any(esp32c5, esp32c61, esp32p4) => 512,
            any(esp32c6, esp32h2) => 256,
            _ => 0,
        }
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

    const PAGE_SIZE: u32 = 0x10000;

    #[cfg(not(esp32s2))]
    pub(super) const ENTRY_COUNT: u32 = {
        const BLOCK: usize = core::mem::size_of::<pac::mmu_table::RegisterBlock>();
        const ENTRY: usize = core::mem::size_of::<pac::mmu_table::ENTRY>();
        (BLOCK / ENTRY) as u32
    };

    pub(super) fn mmu_page_size() -> u32 {
        PAGE_SIZE
    }

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
        f(mmu_table().entry(entry_id as usize).read())
    }

    #[cfg(not(esp32s2))]
    fn write_flash_entry_inner(entry_id: u32, page: u16) {
        mmu_table().entry(entry_id as usize).write(|w| {
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

    pub(super) fn set_entry_invalid(entry_id: u32) {
        mmu_table()
            .entry(entry_id as usize)
            .write(|w| w.invalid().set_bit());
    }

    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        with_entry(entry_id, |entry| !entry.invalid().bit())
    }

    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        with_entry(entry_id, |entry| entry.paddr().bits() as u32)
    }

    #[cfg(not(esp32s2))]
    pub(super) fn write_flash_entry(entry_id: u32, page_paddr: u32) {
        write_flash_entry_inner(entry_id, flash_page_number(page_paddr) as u16);
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

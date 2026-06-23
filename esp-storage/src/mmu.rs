//! Temporary MMU mapping for reading encrypted flash through the cache.
//!
//! Maps a flash physical page into an unused MMU entry, reads decrypted data via
//! the cache, then unmaps and invalidates the cache line.

use core::ptr;

#[cfg(esp32s2)]
use procmacros::ram;

use crate::FlashStorageError;

/// Guard for a temporarily mapped flash MMU page.
pub(crate) struct FlashMmapGuard {
    entry_id: u32,
    vaddr: *const u8,
    page_size: u32,
    /// When `false` the entry was already mapped and must not be invalidated on drop.
    owned: bool,
}

impl FlashMmapGuard {
    pub(crate) fn vaddr(&self) -> *const u8 {
        self.vaddr
    }
}

/// Map a flash physical address to a virtual address for encrypted reading.
pub(crate) fn map_flash_page(paddr: u32) -> Result<FlashMmapGuard, FlashStorageError> {
    let page_size = mmu_page_size();
    let page_paddr = paddr & !(page_size - 1);
    let mmu_val = format_paddr(page_paddr);

    if let Some(entry_id) = find_existing_entry(mmu_val) {
        let vaddr = entry_id_to_vaddr(entry_id);
        if vaddr.is_null() {
            return Err(FlashStorageError::NotSupported);
        }
        return Ok(FlashMmapGuard {
            entry_id,
            vaddr,
            page_size,
            owned: false,
        });
    }

    let entry_id = {
        #[cfg(esp32s2)]
        {
            s2_alloc_entry()
        }
        #[cfg(not(esp32s2))]
        {
            find_free_entry().ok_or(FlashStorageError::NotSupported)?
        }
    };

    #[cfg(esp32s2)]
    {
        let vaddr = s2_map_entry(entry_id, page_paddr)?;
        Ok(FlashMmapGuard {
            entry_id,
            vaddr,
            page_size,
            owned: true,
        })
    }

    #[cfg(not(esp32s2))]
    {
        write_entry(entry_id, mmu_val)?;
        let vaddr = entry_id_to_vaddr(entry_id);
        if vaddr.is_null() {
            set_entry_invalid(entry_id);
            return Err(FlashStorageError::NotSupported);
        }

        Ok(FlashMmapGuard {
            entry_id,
            vaddr,
            page_size,
            owned: true,
        })
    }
}

/// Unmap a temporarily mapped flash page and invalidate the cache.
pub(crate) fn unmap_flash_page(guard: FlashMmapGuard) {
    #[cfg(esp32s2)]
    if guard.owned {
        s2_unmap_entry(guard.entry_id, guard.vaddr as u32, guard.page_size);
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
            Ok::<(), FlashStorageError>(())
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
        if let Some(entry_id) = find_existing_entry(format_paddr(addr)) {
            let vaddr = entry_id_to_vaddr(entry_id);
            if !vaddr.is_null() {
                invalidate_cache(vaddr as u32, page_size);
            }
        }
        addr = addr.saturating_add(page_size);
    }
}

#[cfg(not(esp32))]
fn invalidate_cache(vaddr: u32, size: u32) {
    unsafe {
        unsafe extern "C" {
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
        Cache_Invalidate_Addr(vaddr, size);
    }
}

#[cfg(esp32)]
fn invalidate_cache(_vaddr: u32, _size: u32) {
    unsafe {
        unsafe extern "C" {
            fn Cache_Flush_rom(cpu: u32);
        }
        Cache_Flush_rom(0);
        Cache_Flush_rom(1);
    }
}

fn mmu_page_size() -> u32 {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4) => indexed_mmu_page_size(),
        _ => 0x10000,
    }
}

fn format_paddr(paddr: u32) -> u32 {
    cfg_select! {
        any(esp32c6, esp32h2) => {
            let shift = page_size_shift(indexed_mmu_page_size());
            let mut val = paddr >> shift;
            val |= 1 << 9; // SOC_MMU_VALID
            if esp_hal::efuse::flash_encryption() {
                val |= 1 << 10; // SOC_MMU_SENSITIVE
            }
            val
        }
        any(esp32c5, esp32c61) => {
            let shift = page_size_shift(indexed_mmu_page_size());
            let mut val = paddr >> shift;
            val |= 1 << 10; // SOC_MMU_VALID
            if esp_hal::efuse::flash_encryption() {
                val |= 1 << 11; // SOC_MMU_SENSITIVE
            }
            val
        }
        esp32p4 => {
            let shift = page_size_shift(indexed_mmu_page_size());
            let mut val = paddr >> shift;
            val |= 1 << 12; // SOC_MMU_FLASH_VALID
            if esp_hal::efuse::flash_encryption() {
                val |= 1 << 13; // SOC_MMU_FLASH_SENSITIVE
            }
            val
        }
        _ => paddr >> 16,
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
fn page_size_shift(page_size: u32) -> u32 {
    match page_size {
        0x10000 => 16,
        0x8000 => 15,
        0x4000 => 14,
        0x2000 => 13,
        _ => 16,
    }
}

fn entry_id_to_vaddr(entry_id: u32) -> *const u8 {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2) => {
            (0x4200_0000u32 + entry_id * mmu_page_size()) as *const u8
        }
        esp32p4 => {
            (0x4000_0000u32 + entry_id * mmu_page_size()) as *const u8
        }
        any(esp32c3, esp32c2, esp32s3) => {
            (0x3C00_0000u32 + entry_id * mmu_page_size()) as *const u8
        }
        esp32s2 => s2_entry_id_to_vaddr(entry_id),
        _ => {
            (0x3F40_0000u32 + entry_id * mmu_page_size()) as *const u8
        }
    }
}

/// Virtual address for an S2 MMU entry (DROM/IBUS2 or DBUS0/DBUS1).
///
/// On ESP32-S2, DROM (entries 128..192) is backed by the I-cache bus
/// (`drom0_in_icache = 1` in ESP-IDF). DBUS2/DPORT must not be used here.
#[cfg(esp32s2)]
fn s2_entry_id_to_vaddr(entry_id: u32) -> *const u8 {
    let page_size = mmu_page_size();
    // Matches `mmu_ll_entry_id_to_vaddr_base()` for data-bus entries.
    // Do not use IBUS0/1/2 (entries 0..128): those are not reachable with 8-bit
    // loads. Do not use DBUS2/DPORT (entries 320..384): 32-bit access only.
    let relative = match entry_id {
        0x80..0xC0 => entry_id - 0x80,
        0xC0..0x100 => entry_id - 0xC0,
        0x100..0x140 => entry_id - 0x100,
        _ => return core::ptr::null(),
    };
    (0x3F00_0000u32 + relative * page_size) as *const u8
}

/// Data-bus MMU entry range usable for 8-bit encrypted flash reads on ESP32-S2.
#[cfg(esp32s2)]
const S2_DATA_ENTRY_START: u32 = 0x200 / 4;
#[cfg(esp32s2)]
const S2_DATA_ENTRY_END: u32 = 0x500 / 4;
/// Dedicated scratch slot on the DROM / IBUS2 bus (entry 191 reserved for the bootloader).
#[cfg(esp32s2)]
const S2_TEMP_ENTRY_ID: u32 = 0x200 / 4 + 64 - 2;
#[cfg(esp32s2)]
const S2_IBUS_ENTRY_END: u32 = 0x300 / 4;

/// Pick a free MMU slot, preferring IBUS2/DROM entries first.
#[cfg(esp32s2)]
fn s2_alloc_entry() -> u32 {
    for entry_id in (S2_DATA_ENTRY_START..S2_IBUS_ENTRY_END.saturating_sub(1)).rev() {
        if !entry_is_valid(entry_id) {
            return entry_id;
        }
    }
    find_free_entry().unwrap_or(S2_TEMP_ENTRY_ID)
}

/// Returns `true` for DROM MMU slots (128..192) which live on the I-cache bus on ESP32-S2.
#[cfg(esp32s2)]
fn s2_entry_uses_ibus(entry_id: u32) -> bool {
    (S2_DATA_ENTRY_START..S2_IBUS_ENTRY_END).contains(&entry_id)
}

/// Program one MMU slot through the correct ROM helper (I-bus vs D-bus).
#[cfg(esp32s2)]
#[ram]
fn s2_mmu_rom_set(entry_id: u32, access: u32, vaddr: u32, paddr: u32) -> i32 {
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
        if s2_entry_uses_ibus(entry_id) {
            Cache_Ibus_MMU_Set(access, vaddr, paddr, 64, 1, 0)
        } else {
            Cache_Dbus_MMU_Set(access, vaddr, paddr, 64, 1, 0)
        }
    }
}

/// Map an MMU entry to a flash page via ROM (must run from IRAM on ESP32-S2).
#[cfg(esp32s2)]
#[ram]
fn s2_map_entry(entry_id: u32, page_paddr: u32) -> Result<*const u8, FlashStorageError> {
    let vaddr = s2_entry_id_to_vaddr(entry_id);
    if vaddr.is_null() {
        return Err(FlashStorageError::NotSupported);
    }
    let page_size = mmu_page_size();
    let vaddr_u32 = vaddr as u32;
    invalidate_cache(vaddr_u32, page_size);
    let rc = s2_mmu_rom_set(entry_id, 1 << 15, vaddr_u32, page_paddr);
    if rc != 0 {
        return Err(FlashStorageError::NotSupported);
    }
    invalidate_cache(vaddr_u32, page_size);
    Ok(vaddr)
}

/// Drop cached lines for a temporary mapping. The MMU slot is left mapped so the next
/// remap can overwrite it without touching unrelated I/D-cache state.
#[cfg(esp32s2)]
#[ram]
fn s2_unmap_entry(_entry_id: u32, vaddr: u32, page_size: u32) {
    invalidate_cache(vaddr, page_size);
}

fn find_existing_entry(mmu_val: u32) -> Option<u32> {
    let (start, end) = find_existing_entry_scan_range();
    let paddr_bits = mmu_val & paddr_mask();

    for entry_id in start..end {
        if !entry_is_valid(entry_id) {
            continue;
        }
        if !entry_is_flash_mapping(entry_id) {
            continue;
        }
        if (read_entry(entry_id) & paddr_mask()) == paddr_bits {
            return Some(entry_id);
        }
    }
    None
}

fn find_existing_entry_scan_range() -> (u32, u32) {
    cfg_select! {
        esp32s2 => (S2_DATA_ENTRY_START, S2_DATA_ENTRY_END),
        _ => mmu_entry_scan_range(),
    }
}

/// Returns `true` when the MMU entry maps flash (not PSRAM/SPIRAM).
fn entry_is_flash_mapping(_entry_id: u32) -> bool {
    cfg_select! {
        esp32s2 => (read_entry(_entry_id) & (1 << 15)) != 0, // SOC_MMU_ACCESS_FLASH
        esp32s3 => (read_entry(_entry_id) & (1 << 15)) == 0, // flash (SPIRAM uses bit 15)
        any(esp32c5, esp32c61) => (read_entry(_entry_id) & (1 << 9)) == 0, // SOC_MMU_ACCESS_SPIRAM
        _ => true,
    }
}

fn find_free_entry() -> Option<u32> {
    let (start, end) = mmu_entry_scan_range();
    // Skip the last entry — reserved for bootloader internal flash access.
    (start..end.saturating_sub(1))
        .rev()
        .find(|&entry_id| !entry_is_valid(entry_id))
}

fn paddr_mask() -> u32 {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2) => 0x1ff,
        esp32p4 => 0x3ff,
        any(esp32c3, esp32) => 0xff,
        esp32c2 => 0x3f,
        _ => 0x3fff,
    }
}

fn mmu_entry_scan_range() -> (u32, u32) {
    cfg_select! {
        any(esp32c5, esp32c61, esp32p4) => (0, mmu_entry_count()),
        any(esp32c6, esp32h2) => (0, mmu_entry_count()),
        esp32c3 => (0, 128),
        esp32c2 => (0, 64),
        esp32s3 => (0, 512),
        esp32s2 => (S2_DATA_ENTRY_START, S2_DATA_ENTRY_END),
        _ => (0, 64),
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
fn mmu_entry_count() -> u32 {
    cfg_select! {
        any(esp32c5, esp32c61) => 512,
        any(esp32c6, esp32h2) => 256,
        esp32p4 => 512,
        _ => 0,
    }
}

fn entry_is_valid(entry_id: u32) -> bool {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4) => {
            (read_entry(entry_id) & valid_bit()) != 0
        }
        any(esp32c3, esp32) => {
            (read_entry(entry_id) & (1 << 8)) == 0
        }
        esp32c2 => {
            (read_entry(entry_id) & (1 << 6)) == 0
        }
        _ => {
            (read_entry(entry_id) & (1 << 14)) == 0
        }
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
fn valid_bit() -> u32 {
    cfg_select! {
        any(esp32c6, esp32h2) => 1 << 9,
        any(esp32c5, esp32c61) => 1 << 10,
        esp32p4 => 1 << 12,
        _ => 0,
    }
}

fn set_entry_invalid(entry_id: u32) {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4) => write_indexed_entry(entry_id, 0),
        any(esp32c3, esp32) => write_table_entry(entry_id, 1 << 8),
        esp32c2 => write_table_entry(entry_id, 1 << 6),
        esp32s2 => write_table_entry(entry_id, 1 << 14),
        esp32s3 => write_table_entry(entry_id, 1 << 14),
    }
}

#[cfg(not(esp32s2))]
fn write_entry(entry_id: u32, mmu_val: u32) -> Result<(), FlashStorageError> {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4) => {
            write_indexed_entry(entry_id, mmu_val);
            Ok(())
        }
        _ => {
            write_table_entry(entry_id, mmu_val);
            Ok(())
        }
    }
}

fn read_entry(entry_id: u32) -> u32 {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2) => read_indexed_entry(0x6000_2000, entry_id),
        esp32p4 => read_indexed_entry(0x5008_C000, entry_id),
        any(esp32c3, esp32c2, esp32s3) => read_table_entry(0x600C_5000, entry_id),
        esp32s2 => read_table_entry(0x6180_1000, entry_id),
        _ => read_table_entry(0x3FF1_0000, entry_id),
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
#[inline(always)]
fn read_indexed_entry(base: u32, entry_id: u32) -> u32 {
    unsafe {
        (base as *mut u32).add(0x380 / 4).write_volatile(entry_id);
        (base as *const u32).add(0x37C / 4).read_volatile()
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
#[inline(always)]
fn write_indexed_entry(entry_id: u32, content: u32) {
    cfg_select! {
        any(esp32c5, esp32c6, esp32c61, esp32h2) => {
            let base = 0x6000_2000u32;
            unsafe {
                (base as *mut u32).add(0x380 / 4).write_volatile(entry_id);
                (base as *mut u32).add(0x37C / 4).write_volatile(content);
            }
        }
        _ => {
            let base = 0x5008_C000u32;
            unsafe {
                (base as *mut u32).add(0x380 / 4).write_volatile(entry_id);
                (base as *mut u32).add(0x37C / 4).write_volatile(content);
            }
        }
    }
}

#[cfg(not(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4)))]
#[inline(always)]
fn read_table_entry(base: u32, entry_id: u32) -> u32 {
    unsafe { (base as *const u32).add(entry_id as usize).read_volatile() }
}

#[cfg(not(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4)))]
#[inline(always)]
fn write_table_entry(entry_id: u32, content: u32) {
    cfg_select! {
        any(esp32c3, esp32c2, esp32s3) => {
            let base = 0x600C_5000u32;
            unsafe {
                (base as *mut u32).add(entry_id as usize).write_volatile(content);
            }
        }
        esp32s2 => {
            let base = 0x6180_1000u32;
            unsafe {
                (base as *mut u32).add(entry_id as usize).write_volatile(content);
            }
        }
        _ => {
            let base = 0x3FF1_0000u32;
            unsafe {
                (base as *mut u32).add(entry_id as usize).write_volatile(content);
            }
        }
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2, esp32p4))]
fn indexed_mmu_page_size() -> u32 {
    let code = indexed_mmu_page_size_code();
    match code {
        0 => 0x10000,
        1 => 0x8000,
        2 => 0x4000,
        _ => 0x2000,
    }
}

#[cfg(any(esp32c5, esp32c6, esp32c61, esp32h2))]
fn indexed_mmu_page_size_code() -> u32 {
    let base = 0x6000_2000u32;
    let ctrl = unsafe { (base as *const u32).add(0x384 / 4).read_volatile() };
    (ctrl >> 3) & 0x3
}

#[cfg(esp32p4)]
fn indexed_mmu_page_size_code() -> u32 {
    let base = 0x5008_C000u32;
    let ctrl = unsafe { (base as *const u32).add(0x384 / 4).read_volatile() };
    (ctrl >> 3) & 0x3
}

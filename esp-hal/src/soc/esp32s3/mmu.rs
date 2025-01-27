//! Thin MMU bindings
//!
//! More general information about the MMU can be found here:
//! <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/mm.html#introduction>

const DBUS_VADDR_BASE: u32 = 0x3C000000;
const DR_REG_MMU_TABLE: u32 = 0x600C5000;
const ENTRY_ACCESS_FLASH: u32 = 0;
const ENTRY_INVALID: u32 = 1 << 14;
const ENTRY_TYPE: u32 = 1 << 15;
const ENTRY_VALID: u32 = 0;
const ENTRY_VALID_VAL_MASK: u32 = 0x3fff;
const ICACHE_MMU_SIZE: usize = 0x800;

pub(super) const ENTRY_ACCESS_SPIRAM: u32 = 1 << 15;
pub(super) const PAGE_SIZE: usize = 0x10000;
pub(super) const TABLE_SIZE: usize = ICACHE_MMU_SIZE / core::mem::size_of::<u32>();

extern "C" {
    /// Set DCache mmu mapping.
    ///
    /// [`ext_ram`]: u32 ENTRY_ACCESS_FLASH for flash, ENTRY_ACCESS_SPIRAM for spiram, ENTRY_INVALID for invalid.
    /// [`vaddr`]: u32 Virtual address in CPU address space.
    /// [`paddr`]: u32 Physical address in external memory. Should be aligned by psize.
    /// [`psize`]: u32 Page size of DCache, in kilobytes. Should be 64 here.
    /// [`num`]: u32 Pages to be set.
    /// [`fixes`]: u32 0 for physical pages grow with virtual pages, other for virtual pages map to same physical page.
    pub(super) fn cache_dbus_mmu_set(
        ext_ram: u32,
        vaddr: u32,
        paddr: u32,
        psize: u32,
        num: u32,
        fixed: u32,
    ) -> i32;

    fn Cache_Invalidate_Addr(addr: u32, size: u32);
    fn Cache_WriteBack_All();
    fn rom_Cache_WriteBack_Addr(addr: u32, size: u32);
}

#[procmacros::ram]
pub(super) fn last_mapped_index() -> Option<usize> {
    let mmu_table_ptr = DR_REG_MMU_TABLE as *const u32;
    (0..TABLE_SIZE)
        .rev()
        .find(|&i| unsafe { mmu_table_ptr.add(i).read_volatile() } != ENTRY_INVALID)
}

#[procmacros::ram]
pub(super) fn index_to_data_address(index: usize) -> u32 {
    DBUS_VADDR_BASE + (PAGE_SIZE * index) as u32
}

/// Count flash-mapped pages, de-duplicating mappings which refer to flash page
/// 0
#[procmacros::ram]
pub(super) fn count_effective_flash_pages() -> usize {
    let mmu_table_ptr = DR_REG_MMU_TABLE as *const u32;
    let mut page0_seen = false;
    let mut flash_pages = 0;
    for i in 0..(TABLE_SIZE - 1) {
        let mapping = unsafe { mmu_table_ptr.add(i).read_volatile() };
        if mapping & (ENTRY_INVALID | ENTRY_TYPE) == ENTRY_VALID | ENTRY_ACCESS_FLASH {
            if mapping & ENTRY_VALID_VAL_MASK == 0 {
                if page0_seen {
                    continue;
                }
                page0_seen = true;
            }
            flash_pages += 1;
        }
    }
    flash_pages
}

#[procmacros::ram]
unsafe fn move_flash_to_psram_with_spare(
    target_entry: usize,
    psram_page: usize,
    spare_entry: usize,
) {
    let mmu_table_ptr = DR_REG_MMU_TABLE as *mut u32;
    let target_entry_addr = DBUS_VADDR_BASE + (target_entry * PAGE_SIZE) as u32;
    let spare_entry_addr = DBUS_VADDR_BASE + (spare_entry * PAGE_SIZE) as u32;
    unsafe {
        mmu_table_ptr
            .add(spare_entry)
            .write_volatile(psram_page as u32 | ENTRY_ACCESS_SPIRAM);
        Cache_Invalidate_Addr(spare_entry_addr, PAGE_SIZE as u32);
        core::ptr::copy_nonoverlapping(
            target_entry_addr as *const u8,
            spare_entry_addr as *mut u8,
            PAGE_SIZE,
        );
        rom_Cache_WriteBack_Addr(spare_entry_addr, PAGE_SIZE as u32);
        mmu_table_ptr
            .add(target_entry)
            .write_volatile(psram_page as u32 | ENTRY_ACCESS_SPIRAM);
    }
}

/// Copy flash-mapped pages to PSRAM, copying flash-page 0 only once, and re-map
/// those pages to the PSRAM copies
#[procmacros::ram]
pub(super) unsafe fn copy_flash_to_psram_and_remap(free_page: usize) -> usize {
    let mmu_table_ptr = DR_REG_MMU_TABLE as *mut u32;

    const SPARE_PAGE: usize = TABLE_SIZE - 1;
    const SPARE_PAGE_DCACHE_ADDR: u32 = DBUS_VADDR_BASE + (SPARE_PAGE * PAGE_SIZE) as u32;

    let spare_page_mapping = unsafe { mmu_table_ptr.add(SPARE_PAGE).read_volatile() };
    let mut page0_page = None;
    let mut psram_page = free_page;

    unsafe { Cache_WriteBack_All() };
    for i in 0..(TABLE_SIZE - 1) {
        let mapping = unsafe { mmu_table_ptr.add(i).read_volatile() };
        if mapping & (ENTRY_INVALID | ENTRY_TYPE) != ENTRY_VALID | ENTRY_ACCESS_FLASH {
            continue;
        }
        if mapping & ENTRY_VALID_VAL_MASK == 0 {
            match page0_page {
                Some(page) => {
                    unsafe {
                        mmu_table_ptr
                            .add(i)
                            .write_volatile(page as u32 | ENTRY_ACCESS_SPIRAM)
                    };
                    continue;
                }
                None => page0_page = Some(psram_page),
            }
        }
        unsafe { move_flash_to_psram_with_spare(i, psram_page, SPARE_PAGE) };
        psram_page += 1;
    }

    // Restore spare page mapping
    unsafe {
        mmu_table_ptr
            .add(SPARE_PAGE)
            .write_volatile(spare_page_mapping);
        Cache_Invalidate_Addr(SPARE_PAGE_DCACHE_ADDR, PAGE_SIZE as u32);
    }

    // Special handling if the spare page was mapped to flash
    if spare_page_mapping & (ENTRY_INVALID | ENTRY_TYPE) == ENTRY_VALID | ENTRY_ACCESS_FLASH {
        unsafe {
            // We're running from ram so using the first page should not cause issues
            const SECOND_SPARE: usize = 0;
            let second_spare_mapping = mmu_table_ptr.add(SECOND_SPARE).read_volatile();

            move_flash_to_psram_with_spare(SPARE_PAGE, psram_page, SECOND_SPARE);

            // Restore spare page mapping
            mmu_table_ptr.add(0).write_volatile(second_spare_mapping);
            Cache_Invalidate_Addr(
                DBUS_VADDR_BASE + (SECOND_SPARE * PAGE_SIZE) as u32,
                PAGE_SIZE as u32,
            );
        }
        psram_page += 1;
    }
    psram_page - free_page
}

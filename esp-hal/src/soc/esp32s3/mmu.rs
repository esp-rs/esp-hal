//! Thin MMU bindings
//!
//! More general information about the MMU can be found here:
//! https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/mm.html#introduction

const DBUS_VADDR_BASE: u32 = 0x3C000000;
const DR_REG_MMU_TABLE: u32 = 0x600C5000;
const ENTRY_INVALID: u32 = 1 << 14;
const ICACHE_MMU_SIZE: usize = 0x800;
const TABLE_SIZE: usize = ICACHE_MMU_SIZE / core::mem::size_of::<u32>();

pub(super) const ENTRY_ACCESS_SPIRAM: u32 = 1 << 15;
pub(super) const PAGE_SIZE: usize = 0x10000;

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

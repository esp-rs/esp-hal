//! Cache suspend/resume and MMU-backed invalidation for flash operations.

use procmacros::ram;

/// Suspends the flash-backed caches for the duration of a ROM flash operation.
pub(super) struct CacheGuard {
    inner: Inner,
}

cfg_select! {
    esp32 => {
        struct Inner {
            pro: bool,
            app: bool,
        }
    }
    any(esp32s2, esp32s3) => {
        struct Inner {
            icache: u32,
            dcache: u32,
        }
    }
    esp32p4 => {
        // External flash/PSRAM is served by L2. L1 caches internal RAM; suspending
        // it stalls instruction fetch from IRAM (`#[ram]`).
        struct Inner {
            l2: u32,
        }
    }
    esp32s31 => {
        struct Inner {
            i0: u32,
            i1: u32,
            d: u32,
        }
    }
    _ => {
        struct Inner {
            autoload: u32,
        }
    }
}

impl CacheGuard {
    /// Disable the branch predictor (it issues cache requests) and suspend the
    /// external-memory caches.
    #[ram]
    pub(super) fn suspend() -> Self {
        #[cfg(soc_cpu_has_branch_predictor)]
        disable_branch_predictor();

        let inner = suspend_caches();
        Self { inner }
    }

    /// Drop cached lines that still map the given flash physical range.
    #[ram]
    pub(super) fn invalidate_physical(&self, start: u32, len: u32) {
        if len == 0 {
            return;
        }

        cfg_select! {
            esp32 => {
                // The ESP32 cannot invalidate a single address; flush while still off.
                let _ = (self, start);
                unsafe {
                    if self.inner.pro {
                        Cache_Flush_rom(0);
                    }
                    if self.inner.app {
                        Cache_Flush_rom(1);
                    }
                }
            }
            _ => {
                let _ = self;
                invalidate_mapped_range(start, len);
            }
        }
    }
}

impl Drop for CacheGuard {
    #[ram]
    fn drop(&mut self) {
        resume_caches(&self.inner);

        #[cfg(soc_cpu_has_branch_predictor)]
        enable_branch_predictor();
    }
}

#[cfg(soc_cpu_has_branch_predictor)]
#[ram]
fn disable_branch_predictor() {
    // MHCR (0x7c1): RS, BFE, BTB. Same bits as `soc::enable_branch_predictor`.
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrc x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

#[cfg(soc_cpu_has_branch_predictor)]
#[ram]
fn enable_branch_predictor() {
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

cfg_select! {
    esp32 => {
        unsafe extern "C" {
            fn Cache_Flush_rom(cpu: u32);
        }
    }
    esp32s2 => {
        unsafe extern "C" {
            fn Cache_Suspend_ICache() -> u32;
            fn Cache_Suspend_DCache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
            fn Cache_Resume_DCache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
    }
    esp32s3 => {
        unsafe extern "C" {
            fn rom_Cache_Suspend_ICache() -> u32;
            fn rom_Cache_Suspend_DCache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
            fn Cache_Resume_DCache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
    }
    esp32p4 => {
        unsafe extern "C" {
            fn Cache_Suspend_L2_Cache() -> u32;
            fn Cache_Resume_L2_Cache(autoload: u32);
            fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
        }
    }
    esp32s31 => {
        unsafe extern "C" {
            fn Cache_Suspend_L1_CORE0_ICache() -> u32;
            fn Cache_Suspend_L1_CORE1_ICache() -> u32;
            fn Cache_Suspend_L1_DCache() -> u32;
            fn Cache_Resume_L1_DCache(autoload: u32);
            fn Cache_Resume_L1_CORE1_ICache(autoload: u32);
            fn Cache_Resume_L1_CORE0_ICache(autoload: u32);
            fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
        }
    }
    any(esp32c5, esp32c61) => {
        unsafe extern "C" {
            fn Cache_Suspend_Cache() -> u32;
            fn Cache_Resume_Cache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
    }
    _ => {
        unsafe extern "C" {
            fn Cache_Suspend_ICache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
    }
}

#[ram]
fn suspend_caches() -> Inner {
    cfg_select! {
        esp32 => {
            use crate::peripherals::DPORT;
            let dport = DPORT::regs();
            let pro = dport.pro_cache_ctrl().read().pro_cache_enable().bit();
            let app = dport.app_cache_ctrl().read().app_cache_enable().bit();
            if pro {
                while dport.pro_dcache_dbug0().read().pro_cache_state().bits() != 1 {}
                dport
                    .pro_cache_ctrl()
                    .modify(|_, w| w.pro_cache_enable().clear_bit());
            }
            if app {
                while dport.app_dcache_dbug0().read().app_cache_state().bits() != 1 {}
                dport
                    .app_cache_ctrl()
                    .modify(|_, w| w.app_cache_enable().clear_bit());
            }
            let _ = dport.pro_cache_ctrl().read();
            Inner { pro, app }
        }
        esp32s2 => unsafe {
            Inner {
                icache: Cache_Suspend_ICache(),
                dcache: Cache_Suspend_DCache(),
            }
        },
        esp32s3 => {
            // ROM `Cache_Suspend_I/DCache` does not wait until the cache FSM is
            // idle (`ESP_ROM_HAS_CACHE_SUSPEND_WAITI_BUG`).
            use crate::peripherals::EXTMEM;
            let extmem = EXTMEM::regs();
            unsafe {
                let icache = rom_Cache_Suspend_ICache();
                while extmem.cache_state().read().icache_state().bits() != 1 {}
                let dcache = rom_Cache_Suspend_DCache();
                while extmem.cache_state().read().dcache_state().bits() != 1 {}
                Inner { icache, dcache }
            }
        }
        esp32p4 => unsafe {
            Inner {
                l2: Cache_Suspend_L2_Cache(),
            }
        },
        esp32s31 => unsafe {
            Inner {
                i0: Cache_Suspend_L1_CORE0_ICache(),
                i1: Cache_Suspend_L1_CORE1_ICache(),
                d: Cache_Suspend_L1_DCache(),
            }
        },
        any(esp32c5, esp32c61) => unsafe {
            Inner {
                autoload: Cache_Suspend_Cache(),
            }
        },
        _ => unsafe {
            Inner {
                autoload: Cache_Suspend_ICache(),
            }
        },
    }
}

#[ram]
fn resume_caches(inner: &Inner) {
    cfg_select! {
        esp32 => {
            use crate::peripherals::DPORT;
            let dport = DPORT::regs();
            if inner.pro {
                dport
                    .pro_cache_ctrl()
                    .modify(|_, w| w.pro_cache_enable().set_bit());
            }
            if inner.app {
                dport
                    .app_cache_ctrl()
                    .modify(|_, w| w.app_cache_enable().set_bit());
            }
            let _ = dport.pro_cache_ctrl().read();
        }
        any(esp32s2, esp32s3) => unsafe {
            Cache_Resume_DCache(inner.dcache);
            Cache_Resume_ICache(inner.icache);
        },
        esp32p4 => unsafe { Cache_Resume_L2_Cache(inner.l2) },
        esp32s31 => unsafe {
            Cache_Resume_L1_DCache(inner.d);
            Cache_Resume_L1_CORE1_ICache(inner.i1);
            Cache_Resume_L1_CORE0_ICache(inner.i0);
        },
        any(esp32c5, esp32c61) => unsafe { Cache_Resume_Cache(inner.autoload) },
        _ => unsafe { Cache_Resume_ICache(inner.autoload) },
    }
}

#[cfg(not(esp32))]
#[ram]
fn invalidate_mapped_range(start: u32, len: u32) {
    let page_size = mmu_page_size();
    let page_start = start & !(page_size - 1);
    let end = start.saturating_add(len);
    let mut addr = page_start;

    while addr < end {
        if let Some(entry_id) = find_existing_entry(addr) {
            let vaddr = entry_id_to_vaddr(entry_id);
            if !vaddr.is_null() {
                invalidate_cache_addr(vaddr as u32, page_size);
            }
        }
        addr = addr.saturating_add(page_size);
    }
}

#[cfg(not(esp32))]
#[ram]
fn invalidate_cache_addr(vaddr: u32, size: u32) {
    cfg_select! {
        esp32s31 => {
            const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
            const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
            const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
            unsafe {
                Cache_Invalidate_Addr(
                    CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | CACHE_MAP_L1_DCACHE,
                    vaddr,
                    size,
                )
            }
        }
        esp32p4 => {
            const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
            const CACHE_MAP_L2_CACHE: u32 = 1 << 5;
            // Flash is L2; L1 DCache may still hold a copy of a mapped line.
            unsafe { Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE | CACHE_MAP_L2_CACHE, vaddr, size) }
        }
        _ => unsafe { Cache_Invalidate_Addr(vaddr, size) },
    }
}

#[cfg(not(esp32))]
#[ram]
fn mmu_page_size() -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::mmu_page_size(),
        _ => property!("mmu.page_size"),
    }
}

#[cfg(not(esp32))]
#[ram]
fn entry_id_to_vaddr(entry_id: u32) -> *const u8 {
    cfg_select! {
        esp32s2 => s2::entry_id_to_vaddr(entry_id),
        _ => {
            let base = memory_range!("DROM").start as u32;
            (base + entry_id * mmu_page_size()) as *const u8
        }
    }
}

#[cfg(not(esp32))]
#[ram]
fn find_existing_entry(page_paddr: u32) -> Option<u32> {
    let page = flash_page_number(page_paddr);
    let (start, end) = mmu_entry_scan_range();

    (start..end).find(|&entry_id| {
        entry_is_valid(entry_id)
            && entry_is_flash_mapping(entry_id)
            && entry_flash_page(entry_id) == page
    })
}

#[cfg(not(esp32))]
#[ram]
fn mmu_entry_scan_range() -> (u32, u32) {
    cfg_select! {
        not(soc_has_mmu_table) => (0, property!("mmu.entry_num")),
        esp32s2 => (s2::DATA_ENTRY_START, s2::DATA_ENTRY_END),
        _ => (0, property!("mmu.entry_num")),
    }
}

#[cfg(not(esp32))]
#[ram]
fn flash_page_number(page_paddr: u32) -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::flash_page_number(page_paddr),
        _ => page_paddr >> 16,
    }
}

#[cfg(not(esp32))]
#[ram]
fn entry_is_valid(entry_id: u32) -> bool {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::entry_is_valid(entry_id),
        _ => table::entry_is_valid(entry_id),
    }
}

#[cfg(not(esp32))]
#[ram]
fn entry_is_flash_mapping(entry_id: u32) -> bool {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::entry_is_flash_mapping(entry_id),
        _ => table::entry_is_flash_mapping(entry_id),
    }
}

#[cfg(not(esp32))]
#[ram]
fn entry_flash_page(entry_id: u32) -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::entry_flash_page(entry_id),
        _ => table::entry_flash_page(entry_id),
    }
}

#[cfg(all(not(esp32), not(soc_has_mmu_table)))]
mod indexed {
    use procmacros::ram;

    use crate::peripherals::SPI0;

    #[ram]
    pub(super) fn mmu_page_size() -> u32 {
        let code = mmu_page_size_code();
        cfg_select! {
            esp32s31 => 0x40000 >> code,
            _ => 0x10000 >> code,
        }
    }

    #[ram]
    fn mmu_page_size_code() -> u32 {
        let ctrl = SPI0::regs().mmu_power_ctrl().read();
        cfg_select! {
            any(esp32c5, esp32c61, esp32s31) => ctrl.mmu_page_size().bits() as u32,
            _ => ctrl.spi_mmu_page_size().bits() as u32,
        }
    }

    #[ram]
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

    #[ram]
    pub(super) fn flash_page_number(page_paddr: u32) -> u32 {
        page_paddr >> page_size_shift(mmu_page_size())
    }

    #[ram]
    fn with_entry<R>(entry_id: u32, f: impl FnOnce() -> R) -> R {
        SPI0::regs()
            .mmu_item_index()
            .write(|w| unsafe { w.mmu_item_index().bits(entry_id) });
        f()
    }

    #[ram]
    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        with_entry(entry_id, || {
            SPI0::regs().mmu_item_content().read().valid().bit()
        })
    }

    #[ram]
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

    #[ram]
    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        with_entry(entry_id, || {
            SPI0::regs().mmu_item_content().read().paddr().bits() as u32
        })
    }
}

#[cfg(all(not(esp32), soc_has_mmu_table))]
mod table {
    use procmacros::ram;

    use crate::peripherals::MMU_TABLE;

    #[ram]
    pub(super) fn entry_is_valid(entry_id: u32) -> bool {
        !MMU_TABLE::regs()
            .entry(entry_id as usize)
            .read()
            .invalid()
            .bit()
    }

    #[ram]
    pub(super) fn entry_is_flash_mapping(entry_id: u32) -> bool {
        let e = MMU_TABLE::regs().entry(entry_id as usize).read();
        cfg_select! {
            esp32s2 => e.access_flash().bit(),
            esp32s3 => !e.access_spiram().bit(),
            _ => {
                let _ = e;
                true
            }
        }
    }

    #[ram]
    pub(super) fn entry_flash_page(entry_id: u32) -> u32 {
        MMU_TABLE::regs()
            .entry(entry_id as usize)
            .read()
            .paddr()
            .bits() as u32
    }
}

#[cfg(esp32s2)]
mod s2 {
    pub(super) const DATA_ENTRY_START: u32 = 0x200 / 4;
    pub(super) const DATA_ENTRY_END: u32 = 0x500 / 4;

    pub(super) fn entry_id_to_vaddr(entry_id: u32) -> *const u8 {
        let page_size = super::mmu_page_size();
        let relative = match entry_id {
            0x80..0xC0 => entry_id - 0x80,
            0xC0..0x100 => entry_id - 0xC0,
            0x100..0x140 => entry_id - 0x100,
            _ => return core::ptr::null(),
        };
        (0x3F00_0000u32 + relative * page_size) as *const u8
    }
}

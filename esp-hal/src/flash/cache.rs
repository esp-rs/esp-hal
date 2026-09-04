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

    /// Flush every line. Must run while the caches are still off.
    #[cfg(esp32)]
    #[ram]
    pub(super) fn flush_while_off(&self) {
        unsafe {
            if self.inner.pro {
                Cache_Flush_rom(0);
            }
            if self.inner.app {
                Cache_Flush_rom(1);
            }
        }
    }
}

/// Drop cache lines that map `start..start+len` after the caches are back on.
///
/// IDF invalidates in this order (`spi_flash_check_and_flush_cache` after the
/// operation’s cache-restore path). ESP32 cannot invalidate by address and
/// flushes while still off instead.
#[cfg(not(esp32))]
#[ram]
pub(super) fn invalidate_mapped(start: u32, len: u32) {
    if len == 0 {
        return;
    }
    invalidate_mapped_range(start, len);
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

unsafe extern "C" {
    cfg_select! {
        esp32 => {
            fn Cache_Flush_rom(cpu: u32);
        }
        any(esp32s2, esp32s3) => {
            fn Cache_Suspend_ICache() -> u32;
            fn Cache_Suspend_DCache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
            fn Cache_Resume_DCache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
        esp32p4 => {
            fn Cache_Suspend_L2_Cache() -> u32;
            fn Cache_Resume_L2_Cache(autoload: u32);
            fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
        }
        esp32s31 => {
            fn Cache_Suspend_L1_CORE0_ICache() -> u32;
            fn Cache_Suspend_L1_CORE1_ICache() -> u32;
            fn Cache_Suspend_L1_DCache() -> u32;
            fn Cache_Resume_L1_CORE0_ICache(autoload: u32);
            fn Cache_Resume_L1_CORE1_ICache(autoload: u32);
            fn Cache_Resume_L1_DCache(autoload: u32);
            fn Cache_Invalidate_Addr(map: u32, addr: u32, size: u32);
        }
        any(esp32c5, esp32c61) => {
            fn Cache_Suspend_Cache() -> u32;
            fn Cache_Resume_Cache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
        any(esp32c2, esp32c3, esp32c6, esp32h2) => {
            fn Cache_Suspend_ICache() -> u32;
            fn Cache_Resume_ICache(autoload: u32);
            fn Cache_Invalidate_Addr(addr: u32, size: u32);
        }
        _ => {
            compile_error!("missing flash cache ROM bindings for this chip");
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
        any(esp32s2, esp32s3) => {
            #[cfg(esp32s3)]
            use crate::peripherals::EXTMEM;
            unsafe {
                let icache = Cache_Suspend_ICache();
                // ROM `Cache_Suspend_I/DCache` on ESP32-S3 does not wait until
                // the cache FSM is idle (`ESP_ROM_HAS_CACHE_SUSPEND_WAITI_BUG`).
                #[cfg(esp32s3)]
                while EXTMEM::regs().cache_state().read().icache_state().bits() != 1 {}
                let dcache = Cache_Suspend_DCache();
                #[cfg(esp32s3)]
                while EXTMEM::regs().cache_state().read().dcache_state().bits() != 1 {}
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
#[ram]
fn invalidate_mapped_page(page_paddr: u32, page_size: u32) {
    let page = flash_page_number(page_paddr);
    let (start, end) = mmu_entry_scan_range();

    for entry_id in start..end {
        if entry_is_valid(entry_id)
            && entry_is_flash_mapping(entry_id)
            && entry_flash_page(entry_id) == page
        {
            invalidate_entry_caches(entry_id, page_size);
        }
    }
}

#[cfg(not(esp32))]
#[ram]
fn invalidate_entry_caches(entry_id: u32, page_size: u32) {
    cfg_select! {
        esp32s2 => {
            let vaddr = s2::entry_id_to_vaddr(entry_id);
            if !vaddr.is_null() {
                invalidate_cache_addr(vaddr as u32, page_size);
            }
        }
        _ => {
            let offset = entry_id * page_size;
            let drom = memory_range!("DROM").start as u32 + offset;
            invalidate_cache_addr(drom, page_size);
            let irom = memory_range!("IROM").start as u32 + offset;
            if irom != drom {
                invalidate_cache_addr(irom, page_size);
            }
        }
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
            const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
            const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
            const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
            const CACHE_MAP_L2_CACHE: u32 = 1 << 5;
            // IROM and DROM share the same window; invalidate I, D, and L2.
            unsafe {
                Cache_Invalidate_Addr(
                    CACHE_MAP_L1_ICACHE_0
                        | CACHE_MAP_L1_ICACHE_1
                        | CACHE_MAP_L1_DCACHE
                        | CACHE_MAP_L2_CACHE,
                    vaddr,
                    size,
                )
            }
        }
        _ => unsafe { Cache_Invalidate_Addr(vaddr, size) },
    }
}

#[cfg(not(esp32))]
#[ram]
fn mmu_page_size() -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::mmu_page_size(),
        // C2 page size is configurable (IDF `mmu_ll_get_page_size`).
        esp32c2 => c2::mmu_page_size(),
        _ => property!("mmu.page_size"),
    }
}

#[cfg(not(esp32))]
#[ram]
fn mmu_entry_scan_range() -> (u32, u32) {
    cfg_select! {
        not(soc_has_mmu_table) => (0, property!("mmu.entry_num")),
        // Include I-bus slots (0..0x80). Skip DBUS2/DPORT (0x140..).
        esp32s2 => (0, s2::DATA_ENTRY_END),
        _ => (0, property!("mmu.entry_num")),
    }
}

#[cfg(not(esp32))]
#[ram]
fn flash_page_number(page_paddr: u32) -> u32 {
    cfg_select! {
        not(soc_has_mmu_table) => indexed::flash_page_number(page_paddr),
        esp32c2 => page_paddr >> c2::mmu_page_size().trailing_zeros(),
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

#[cfg(esp32c2)]
mod c2 {
    use procmacros::ram;

    use crate::peripherals::EXTMEM;

    /// 0 = 16 KiB, 1 = 32 KiB, 2 = 64 KiB (`EXTMEM_CACHE_MMU_PAGE_SIZE`).
    #[ram]
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

    pub(super) const DATA_ENTRY_END: u32 = 0x500 / 4;

    #[ram]
    pub(super) fn entry_id_to_vaddr(entry_id: u32) -> *const u8 {
        let page_size = super::mmu_page_size();
        // Matches `mmu_ll_entry_id_to_vaddr_base()`: IBUS0/1 at 0x4000_0000,
        // DROM/DBUS at 0x3F00_0000. Skip DBUS2/DPORT (0x140..).
        let (base, relative) = match entry_id {
            0x00..0x40 => (0x4000_0000u32, entry_id),
            0x40..0x80 => (0x4000_0000u32, entry_id - 0x40),
            0x80..0xC0 => (0x3F00_0000u32, entry_id - 0x80),
            0xC0..0x100 => (0x3F00_0000u32, entry_id - 0xC0),
            0x100..0x140 => (0x3F00_0000u32, entry_id - 0x100),
            _ => return core::ptr::null(),
        };
        (base + relative * page_size) as *const u8
    }
}

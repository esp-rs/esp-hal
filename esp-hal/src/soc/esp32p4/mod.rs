crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod cpu_control;
pub(crate) mod regi2c;

pub(crate) use esp32p4 as pac;

#[inline(always)]
#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {
    // workaround: this shouldn't be needed - done by the 2nd stage bootloader
    unsafe {
        cache_invalidate_addr(0x40000000, 64 * 1024 * 1024);
    }
}

pub(crate) fn pre_init() {
    #[cfg(multi_core)]
    unsafe {
        // Stall Core 1 first (PMU stall), then disable its clock and assert
        // global reset. This undoes any state left by start_core1() that
        // may have survived a software reset, preventing Core 1 from running
        // during the ROM bootloader phase and interfering with espflash.
        cpu_control::internal_park_core(crate::system::Cpu::AppCpu, true);
        cpu_control::disable_core1();
    }
}

pub(crate) fn enable_branch_predictor() {
    // Enable branch predictor
    // Note that the branch predictor will start cache requests and needs to be disabled when
    // the cache is disabled.
    // MHCR: CSR 0x7c1
    const MHCR_RS: u32 = 1 << 4; // R/W, address return stack set bit
    const MHCR_BFE: u32 = 1 << 5; // R/W, allow predictive jump set bit
    const MHCR_BTB: u32 = 1 << 12; // R/W, branch target prediction enable bit
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
const CACHE_MAP_L2_CACHE: u32 = 1 << 5;

// The ROM cache function table, up to the freeze functions. See `cache_internal_stub_table` in
// ESP-IDF `esp_rom/esp32p4/include/esp32p4/rom/cache.h`.
#[cfg(all(idle_frequency_scaling, psram_idle_low_speed_switch))]
#[repr(C)]
struct CacheInternalStubTable {
    _unused: [usize; 20],
    freeze_l2_cache_enable: unsafe extern "C" fn(mode: u32),
    freeze_l2_cache_disable: unsafe extern "C" fn(),
}

#[cfg(all(idle_frequency_scaling, psram_idle_low_speed_switch))]
unsafe extern "C" {
    static rom_cache_internal_table_ptr: *const CacheInternalStubTable;
    fn Cache_WriteBack_All(map: u32) -> i32;
}

/// Freezes the L2 cache, which caches the external memory. An access to flash or PSRAM stalls
/// until [`unfreeze_ext_mem_cache`].
///
/// The caller must run from RAM, and the other core must be stalled.
#[cfg(all(idle_frequency_scaling, psram_idle_low_speed_switch))]
#[inline(always)]
pub(crate) fn freeze_ext_mem_cache() {
    // `CACHE_FREEZE_ACK_BUSY`: a cache miss stalls the requester.
    const ACK_BUSY: u32 = 0;
    unsafe {
        // ESP-IDF writes back the internal memory cache first, to prevent an automatic writeback
        // into the frozen cache.
        Cache_WriteBack_All(CACHE_MAP_L1_DCACHE);
        ((*rom_cache_internal_table_ptr).freeze_l2_cache_enable)(ACK_BUSY);
    }
}

/// Releases the freeze of [`freeze_ext_mem_cache`].
#[cfg(all(idle_frequency_scaling, psram_idle_low_speed_switch))]
#[inline(always)]
pub(crate) fn unfreeze_ext_mem_cache() {
    unsafe { ((*rom_cache_internal_table_ptr).freeze_l2_cache_disable)() };
}

/// Cache buses that back the value at `addr`.
fn cache_l2_bus(addr: u32) -> u32 {
    let internal = memory_range!("DRAM").contains(&addr);
    if internal { 0 } else { CACHE_MAP_L2_CACHE }
}

/// Writes back a specific range of data in the cache.
pub(crate) unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        Cache_WriteBack_Addr(CACHE_MAP_L1_DCACHE | cache_l2_bus(addr), addr, size);
    }
}

/// Invalidate a specific range of data in the cache.
pub(crate) unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE | cache_l2_bus(addr), addr, size);
    }
}

pub(crate) unsafe fn cache_invalidate_icache_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        Cache_Invalidate_Addr(
            CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | cache_l2_bus(addr),
            addr,
            size,
        );
    }
}

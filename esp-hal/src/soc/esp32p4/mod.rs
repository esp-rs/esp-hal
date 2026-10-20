crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
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

const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
const CACHE_MAP_L2_CACHE: u32 = 1 << 5;

/// Cache buses that back the value at `addr`.
fn cache_l2_bus(addr: u32) -> u32 {
    let internal = memory_range!("DRAM").contains(&addr);
    if internal { 0 } else { CACHE_MAP_L2_CACHE }
}

/// Writes back a specific range of data in the cache.
pub(crate) unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    if crate::soc::is_uncached_ram_address(addr as usize) {
        return;
    }

    unsafe extern "C" {
        fn Cache_WriteBack_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        Cache_WriteBack_Addr(CACHE_MAP_L1_DCACHE | cache_l2_bus(addr), addr, size);
    }
}

/// Invalidate a specific range of data in the cache.
pub(crate) unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    if crate::soc::is_uncached_ram_address(addr as usize) {
        return;
    }

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

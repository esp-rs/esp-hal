// pub mod efuse;
pub mod gpio;
// pub mod peripherals;

crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod regi2c;

pub(crate) use esp32p4 as pac;

pub(crate) fn pre_init() {
    // TODO: Check if anything needs to be done here
}

const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
const CACHE_MAP_L2_CACHE: u32 = 1 << 5;

/// Write back a specific range of data in the cache.
pub(crate) unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        Cache_WriteBack_Addr(
            CACHE_MAP_L1_ICACHE_0
                | CACHE_MAP_L1_ICACHE_1
                | CACHE_MAP_L1_DCACHE
                | CACHE_MAP_L2_CACHE,
            addr,
            size,
        );
    }
}

/// Write back a specific range of data in the cache.
pub(crate) unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        Cache_Invalidate_Addr(
            CACHE_MAP_L1_ICACHE_0
                | CACHE_MAP_L1_ICACHE_1
                | CACHE_MAP_L1_DCACHE
                | CACHE_MAP_L2_CACHE,
            addr,
            size,
        );
    }
}

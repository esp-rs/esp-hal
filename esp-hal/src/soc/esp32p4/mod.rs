// pub mod efuse;
pub mod gpio;
// pub mod peripherals;

crate::unstable_module! {
    pub mod clocks;
}

pub(crate) mod cpu_control;
pub(crate) mod regi2c;

pub(crate) use esp32p4 as pac;

#[cfg(spi_master_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn spi_master_clock_source_frequency() -> u32 {
    clocks::apb_clk_frequency()
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

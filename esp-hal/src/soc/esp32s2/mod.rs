//! # SOC (System-on-Chip) module (ESP32-S2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S2` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
    pub mod ulp_core;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32s2 as pac;

#[cfg(i2s_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn i2s_sclk_frequency() -> u32 {
    // I2S uses the 160 MHz PLL tap, derived from either supported PLL frequency.
    match clocks::pll_clk_frequency() {
        320_000_000 => 320_000_000 / 2,
        480_000_000 => 480_000_000 / 3,
        _ => unreachable!(),
    }
}

#[cfg(spi_master_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn spi_master_clock_source_frequency() -> u32 {
    clocks::apb_clk_frequency()
}

/// Write back a specific range of data in the cache.
#[doc(hidden)]
#[unsafe(link_section = ".rwtext")]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_WriteBack_Addr(addr, size);
    }
}

/// Invalidate a specific range of addresses in the cache.
#[doc(hidden)]
#[unsafe(link_section = ".rwtext")]
pub unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(addr, size);
    }
}

/// Get the size of a cache line in the DCache.
#[doc(hidden)]
#[unsafe(link_section = ".rwtext")]
pub unsafe fn cache_get_dcache_line_size() -> u32 {
    unsafe extern "C" {
        fn Cache_Get_DCache_Line_Size() -> u32;
    }
    unsafe { Cache_Get_DCache_Line_Size() }
}

#[crate::ram]
pub(crate) unsafe fn configure_cpu_caches() {
    // Set up caches. Doesn't work when put in `configure_cpu_caches`.
    unsafe extern "C" {
        /// Invalidate all cache items in ICache.
        fn Cache_Invalidate_ICache_All();

        /// Set ICache modes: cache size, associate ways and cache line size.
        ///
        /// @param cache_size_t cache_size : the cache size, can be CACHE_SIZE_HALF and
        /// CACHE_SIZE_FULL
        ///
        /// @param cache_ways_t ways : the associate ways of cache, can only be CACHE_4WAYS_ASSOC
        ///
        /// @param cache_line_size_t cache_line_size : the cache line size, can be
        /// CACHE_LINE_SIZE_16B, CACHE_LINE_SIZE_32B
        fn Cache_Set_ICache_Mode(cache_size: u32, ways: u32, cache_line_size: u32);

        /// Resume ICache access for the cpu.
        ///
        /// @param  uint32_t autoload : ICache will preload then.
        fn Cache_Resume_ICache(autoload: u32);

        /// Allocate memory to used by ICache and DCache.
        ///
        /// [`sram0_layout`]: u32 the usage of first 8KB internal memory block,
        /// can be CACHE_MEMORY_INVALID,
        /// CACHE_MEMORY_ICACHE_LOW, CACHE_MEMORY_ICACHE_HIGH,
        /// CACHE_MEMORY_DCACHE_LOW, CACHE_MEMORY_DCACHE_HIGH
        /// [`sram1_layout`]: the usage of second 8KB internal memory block,
        /// [`sram2_layout`]: the usage of third 8KB internal memory block
        /// [`sram3_layout`]: the usage of forth 8KB internal memory block
        fn Cache_Allocate_SRAM(
            sram0_layout: u32,
            sram1_layout: u32,
            sram2_layout: u32,
            sram3_layout: u32,
        );

        /// Set DCache modes: cache size, associate ways and cache line size.
        ///
        /// [`cache_size`]: u32 the cache size, can be CACHE_SIZE_HALF and CACHE_SIZE_FULL
        /// [`ways`]: u32 the associate ways of cache, can only be CACHE_4WAYS_ASSOC
        /// [`cache_line_size`]: u32 the cache line size, can be CACHE_LINE_SIZE_16B, CACHE_LINE_SIZE_32B
        fn Cache_Set_DCache_Mode(cache_size: u32, ways: u32, cache_line_size: u32);

        /// Invalidate all cache items in DCache.
        fn Cache_Invalidate_DCache_All();

        /// Enable DCache access for the cpu.
        ///
        /// @param  uint32_t autoload : DCache will preload then.
        fn Cache_Enable_DCache(autoload: u32);
    }

    const CACHE_4WAYS_ASSOC: u32 = 0;

    const CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE: u32 = match () {
        _ if cfg!(instruction_cache_size_8kb) => 0,
        _ if cfg!(instruction_cache_size_16kb) => 1,
        _ => core::unreachable!(),
    };
    const CONFIG_ESP32S2_INSTRUCTION_CACHE_LINE_SIZE: u32 = match () {
        _ if cfg!(instruction_cache_line_size_16b) => 0,
        _ if cfg!(instruction_cache_line_size_32b) => 1,
        _ => core::unreachable!(),
    };

    const CONFIG_ESP32S2_DATA_CACHE_SIZE: u32 = match () {
        _ if cfg!(data_cache_size_0kb) => 0, // doesn't matter according to esp-idf
        _ if cfg!(data_cache_size_8kb) => 0,
        _ if cfg!(data_cache_size_16kb) => 1,
        _ => core::unreachable!(),
    };
    const CONFIG_ESP32S2_DATA_CACHE_LINE_SIZE: u32 = match () {
        _ if cfg!(data_cache_line_size_16b) => 0,
        _ if cfg!(data_cache_line_size_32b) => 1,
        _ => core::unreachable!(),
    };

    #[derive(Clone, Copy, Debug)]
    enum CacheLayout {
        Invalid    = 0,
        ICacheLow  = 1 << 0,
        ICacheHigh = 1 << 1,
        DCacheLow  = 1 << 2,
        DCacheHigh = 1 << 3,
    }

    let mut sram = [
        CacheLayout::ICacheLow,
        CacheLayout::Invalid,
        CacheLayout::Invalid,
        CacheLayout::Invalid,
    ];
    let mut idx = 1;

    if cfg!(instruction_cache_size_16kb) {
        sram[idx] = CacheLayout::ICacheHigh;
        idx += 1;
    }

    if !cfg!(data_cache_size_0kb) {
        sram[idx] = CacheLayout::DCacheLow;
        idx += 1;
    }
    if cfg!(data_cache_size_16kb) {
        sram[idx] = CacheLayout::DCacheHigh;
    }

    unsafe {
        Cache_Allocate_SRAM(
            sram[0] as u32,
            sram[1] as u32,
            sram[2] as u32,
            sram[3] as u32,
        );
    }

    unsafe {
        Cache_Set_ICache_Mode(
            CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE,
            CACHE_4WAYS_ASSOC,
            CONFIG_ESP32S2_INSTRUCTION_CACHE_LINE_SIZE,
        );
        Cache_Invalidate_ICache_All();
        Cache_Resume_ICache(0);

        Cache_Set_DCache_Mode(
            CONFIG_ESP32S2_DATA_CACHE_SIZE,
            CACHE_4WAYS_ASSOC,
            CONFIG_ESP32S2_DATA_CACHE_LINE_SIZE,
        );
        Cache_Invalidate_DCache_All();
        Cache_Enable_DCache(0);
    }
}

pub(crate) fn pre_init() {}

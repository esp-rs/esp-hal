//! # SOC (System-on-Chip) module (ESP32-S3)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S3` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
    pub mod ulp_core;
}
#[cfg(feature = "unstable")]
pub mod cpu_control;
pub mod gpio;
pub(crate) mod regi2c;

pub use esp32s3 as pac;

#[cfg(i2s_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn i2s_sclk_frequency() -> u32 {
    clocks::pll_160m_frequency()
}

#[cfg(spi_master_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn spi_master_clock_source_frequency() -> u32 {
    clocks::apb_clk_frequency()
}

#[unsafe(link_section = ".rwtext")]
pub(crate) unsafe fn configure_cpu_caches() {
    // this is just the bare minimum we need to run code from flash
    // consider implementing more advanced configurations
    // see https://github.com/apache/nuttx/blob/f25db331136351dbf8ebe6925a98d3b77e1ca983/arch/xtensa/src/esp32s3/esp32s3_start.c#L213

    unsafe extern "C" {
        fn Cache_Suspend_DCache();

        fn Cache_Resume_DCache(param: u32);

        fn rom_config_instruction_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );

        fn rom_config_data_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );
    }

    const CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE: u32 = match () {
        _ if cfg!(instruction_cache_size_32kb) => 0x8000,
        _ if cfg!(instruction_cache_size_16kb) => 0x4000,
        _ => core::unreachable!(),
    };
    const CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS: u8 = match () {
        _ if cfg!(icache_associated_ways_8) => 8,
        _ if cfg!(icache_associated_ways_4) => 4,
        _ => core::unreachable!(),
    };
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE: u8 = match () {
        _ if cfg!(instruction_cache_line_size_32b) => 32,
        _ if cfg!(instruction_cache_line_size_16b) => 16,
        _ => core::unreachable!(),
    };

    const CONFIG_ESP32S3_DATA_CACHE_SIZE: u32 = match () {
        _ if cfg!(data_cache_size_64kb) => 0x10000,
        _ if cfg!(data_cache_size_32kb) => 0x8000,
        _ if cfg!(data_cache_size_16kb) => 0x4000,
        _ => core::unreachable!(),
    };
    const CONFIG_ESP32S3_DCACHE_ASSOCIATED_WAYS: u8 = match () {
        _ if cfg!(dcache_associated_ways_8) => 8,
        _ if cfg!(dcache_associated_ways_4) => 4,
        _ => core::unreachable!(),
    };
    const CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE: u8 = match () {
        _ if cfg!(data_cache_line_size_64b) => 64,
        _ if cfg!(data_cache_line_size_32b) => 32,
        _ if cfg!(data_cache_line_size_16b) => 16,
        _ => core::unreachable!(),
    };

    // Configure the mode of instruction cache: cache size, cache line size.
    unsafe {
        rom_config_instruction_cache_mode(
            CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE,
            CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS,
            CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE,
        );
    }

    // Configure the mode of data : cache size, cache line size.
    unsafe {
        Cache_Suspend_DCache();
        rom_config_data_cache_mode(
            CONFIG_ESP32S3_DATA_CACHE_SIZE,
            CONFIG_ESP32S3_DCACHE_ASSOCIATED_WAYS,
            CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE,
        );
        Cache_Resume_DCache(0);
    }
}

/// Write back a specific range of data in the cache.
#[doc(hidden)]
#[unsafe(link_section = ".rwtext")]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn rom_Cache_WriteBack_Addr(addr: u32, size: u32);
        fn Cache_Suspend_DCache_Autoload() -> u32;
        fn Cache_Resume_DCache_Autoload(value: u32);
    }
    // suspend autoload, avoid load cachelines being written back
    unsafe {
        let autoload = Cache_Suspend_DCache_Autoload();
        rom_Cache_WriteBack_Addr(addr, size);
        Cache_Resume_DCache_Autoload(autoload);
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

pub(crate) fn pre_init() {}

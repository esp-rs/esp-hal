//! # SOC (System-on-Chip) module (ESP32-S3)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S3` chip.
//!
//! Also few constants are defined in this module for `ESP32-S3` chip:
//!    * I2S_SCLK: 160_000_000 - I2S clock frequency
//!    * I2S_DEFAULT_CLK_SRC: 2 - I2S clock source

crate::unstable_module! {
    pub mod efuse;
    #[cfg(feature = "psram")]
    pub mod psram;
    pub mod trng;
    pub mod ulp_core;
}
pub mod cpu_control;
pub mod gpio;
pub(crate) mod regi2c;

pub use esp32s3 as pac;

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {
    use crate::time::Rate;

    /// The base clock frequency for the I2S peripheral (Hertz).
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for I2S operations.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    /// RMT Clock source value.
    pub const RMT_CLOCK_SRC: u8 = 1;
    /// RMT Clock source frequency.
    pub const RMT_CLOCK_SRC_FREQ: Rate = Rate::from_mhz(80);

    /// A reference clock tick of 1 MHz.
    pub const RC_FAST_CLK: Rate = Rate::from_khz(17500);
}

#[unsafe(link_section = ".rwtext")]
pub(crate) unsafe fn configure_cpu_caches() {
    // this is just the bare minimum we need to run code from flash
    // consider implementing more advanced configurations
    // see https://github.com/apache/incubator-nuttx/blob/master/arch/xtensa/src/esp32s3/esp32s3_start.c

    unsafe extern "C" {
        fn rom_config_instruction_cache_mode(
            cfg_cache_size: u32,
            cfg_cache_ways: u8,
            cfg_cache_line_size: u8,
        );
    }

    // ideally these should be configurable
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE: u32 = 0x8000; // ESP32S3_INSTRUCTION_CACHE_32KB
    const CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS: u8 = 8; // ESP32S3_INSTRUCTION_CACHE_8WAYS
    const CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE: u8 = 32; // ESP32S3_INSTRUCTION_CACHE_LINE_32B

    // Configure the mode of instruction cache: cache size, cache line size.
    unsafe {
        rom_config_instruction_cache_mode(
            CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE,
            CONFIG_ESP32S3_ICACHE_ASSOCIATED_WAYS,
            CONFIG_ESP32S3_INSTRUCTION_CACHE_LINE_SIZE,
        );
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

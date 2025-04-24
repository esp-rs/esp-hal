//! # SOC (System-on-Chip) module (ESP32-S2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-S2` chip.
//!
//! Also few constants are defined in this module for `ESP32-S2` chip:
//!    * I2S_SCLK: 160_000_000 - I2S clock frequency
//!    * I2S_DEFAULT_CLK_SRC: 2 - I2S clock source

use core::ptr::addr_of_mut;

use crate::rtc_cntl::SocResetReason;

crate::unstable_module! {
    pub mod efuse;
    #[cfg(feature = "psram")]
    pub mod psram;
    pub mod trng;
    pub mod ulp_core;
}
pub mod gpio;
pub mod peripherals;
pub(crate) mod regi2c;

/// The name of the chip ("esp32s2") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32s2"
    };
}

/// A link to the Technical Reference Manual (TRM) for the chip.
#[doc(hidden)]
#[macro_export]
macro_rules! trm_link {
    () => { "https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf" };
}

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {
    use crate::time::Rate;

    /// System clock frequency for the I2S peripheral, in Hertz.
    pub const I2S_SCLK: u32 = 160_000_000;
    /// Default clock source for the I2S peripheral.
    pub const I2S_DEFAULT_CLK_SRC: u32 = 2;
    /// Start address of the RMT (Remote Control) peripheral's RAM.
    pub const RMT_RAM_START: usize = 0x3f416400;
    /// The size (number of pulse codes) of each RMT channel's dedicated RAM.
    pub const RMT_CHANNEL_RAM_SIZE: usize = 64;
    /// Reference clock tick frequency, set to 1 MHz.
    pub const REF_TICK: Rate = Rate::from_mhz(1);
}

/// Function initializes ESP32 specific memories (RTC slow and fast) and
/// then calls original Reset function
///
/// ENTRY point is defined in memory.x
/// *Note: the pre_init function is called in the original reset handler
/// after the initializations done in this function*
#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ESP32Reset() -> ! {
    // These symbols come from `memory.x`
    unsafe extern "C" {
        static mut _rtc_fast_bss_start: u32;
        static mut _rtc_fast_bss_end: u32;
        static mut _rtc_fast_persistent_start: u32;
        static mut _rtc_fast_persistent_end: u32;

        static mut _rtc_slow_bss_start: u32;
        static mut _rtc_slow_bss_end: u32;
        static mut _rtc_slow_persistent_start: u32;
        static mut _rtc_slow_persistent_end: u32;

        static mut _stack_start_cpu0: u32;

        static mut __stack_chk_guard: u32;
    }

    // set stack pointer to end of memory: no need to retain stack up to this point
    unsafe {
        xtensa_lx::set_stack_pointer(addr_of_mut!(_stack_start_cpu0));
    }

    // copying data from flash to various data segments is done by the bootloader
    // initialization to zero needs to be done by the application

    // Initialize RTC RAM
    unsafe {
        xtensa_lx_rt::zero_bss(
            addr_of_mut!(_rtc_fast_bss_start),
            addr_of_mut!(_rtc_fast_bss_end),
        );
        xtensa_lx_rt::zero_bss(
            addr_of_mut!(_rtc_slow_bss_start),
            addr_of_mut!(_rtc_slow_bss_end),
        );
    }
    if matches!(
        crate::system::reset_reason(),
        None | Some(SocResetReason::ChipPowerOn)
    ) {
        unsafe {
            xtensa_lx_rt::zero_bss(
                addr_of_mut!(_rtc_fast_persistent_start),
                addr_of_mut!(_rtc_fast_persistent_end),
            );
            xtensa_lx_rt::zero_bss(
                addr_of_mut!(_rtc_slow_persistent_start),
                addr_of_mut!(_rtc_slow_persistent_end),
            );
        }
    }

    let stack_chk_guard = core::ptr::addr_of_mut!(__stack_chk_guard);
    // we _should_ use a random value but we don't have a good source for random
    // numbers here
    unsafe {
        stack_chk_guard.write_volatile(esp_config::esp_config_int!(
            u32,
            "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
        ));
    }

    crate::interrupt::setup_interrupts();

    // continue with default reset handler
    unsafe { xtensa_lx_rt::Reset() }
}

/// The ESP32 has a first stage bootloader that handles loading program data
/// into the right place therefore we skip loading it again.
#[doc(hidden)]
#[unsafe(no_mangle)]
#[rustfmt::skip]
pub extern "Rust" fn __init_data() -> bool {
    false
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

pub(crate) fn pre_init() {}

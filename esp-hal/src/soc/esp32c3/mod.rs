//! # SOC (System-on-Chip) module (ESP32-C3)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C3` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
}
pub(crate) mod regi2c;

pub(crate) use esp32c3 as pac;

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}
pub(crate) fn pre_init() {}

/// Invalidate a specific range of addresses in the cache.
#[cfg(feature = "unstable")]
#[crate::ram]
pub(crate) unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(addr, size);
    }
}

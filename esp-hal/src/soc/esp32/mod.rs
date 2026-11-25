//! # SOC (System-on-Chip) module (ESP32)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
    // The light sleep path stalls the other core, and it is not gated on the feature.
    pub mod cpu_control;
}
#[cfg(all(cache_livelock_workaround, feature = "rt"))]
pub(crate) mod cache_livelock;
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32 as pac;

pub(crate) unsafe fn configure_cpu_caches() {}

pub(crate) fn pre_init() {
    // A debugger reset can leave the APP CPU running ROM code. That code fails when the
    // application uses the memory of the ROM. Matches ESP-IDF's single-core startup.
    crate::peripherals::DPORT::regs()
        .appcpu_ctrl_b()
        .modify(|_, w| w.appcpu_clkgate_en().clear_bit());
}

//! # SOC (System-on-Chip) module (ESP32-C2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C2` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32c2 as pac;

#[cfg(spi_master_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn spi_master_clock_source_frequency() -> u32 {
    clocks::apb_clk_frequency()
}

pub(crate) fn pre_init() {}

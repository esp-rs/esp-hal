//! # SOC (System-on-Chip) module (ESP32)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32` chip.

crate::unstable_module! {
    pub mod clocks;
    pub mod trng;
}
#[cfg(feature = "unstable")]
pub mod cpu_control;
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32 as pac;

#[cfg(i2s_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn i2s_sclk_frequency() -> u32 {
    clocks::pll_f160m_clk_frequency()
}

#[cfg(spi_master_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn spi_master_clock_source_frequency() -> u32 {
    clocks::apb_clk_frequency()
}

pub(crate) unsafe fn configure_cpu_caches() {}

pub(crate) fn pre_init() {}

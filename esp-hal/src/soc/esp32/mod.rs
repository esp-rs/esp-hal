//! # SOC (System-on-Chip) module (ESP32)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32` chip.

crate::unstable_module! {
    pub mod efuse;
    #[cfg(feature = "psram")]
    pub mod psram;
    pub mod trng;
}
pub mod cpu_control;
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32 as pac;

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {
    use crate::time::Rate;

    /// The base clock frequency for the I2S peripheral (Hertz).
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for I2S operations.
    pub const I2S_DEFAULT_CLK_SRC: u32 = 2;
    /// A reference clock tick of 1 MHz.
    pub const REF_TICK: Rate = Rate::from_mhz(1);
}

pub(crate) unsafe fn configure_cpu_caches() {}

pub(crate) fn pre_init() {}

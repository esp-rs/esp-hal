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

pub(crate) fn pre_init() {}

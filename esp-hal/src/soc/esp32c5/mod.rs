//! # SOC (System-on-Chip) module (ESP32-C5)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C5` chip.
//!
//! Also few constants are defined in this module for `ESP32-C5` chip:

crate::unstable_module! {
    pub mod clocks;
    pub mod lp_core;
    pub mod trng;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32c5 as pac;

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {}

pub(crate) fn pre_init() {
    todo!();
}

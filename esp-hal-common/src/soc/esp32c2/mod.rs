//! # SOC (System-on-Chip) module (ESP32-C2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C2` chip.

pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

pub(crate) mod constants {
    pub const SOC_DRAM_LOW: u32 = 0x3FCA_0000;
    pub const SOC_DRAM_HIGH: u32 = 0x3FCE_0000;
}

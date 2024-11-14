//! # SOC (System-on-Chip) module (ESP32-P4)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-P4` chip.

pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod trng;

/// The name of the chip ("esp32h2") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32p4"
    };
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x500D_6000;
    pub const INTERRUPT_MAP_BASE_APP_CPU: u32 = 0x0; // FIXME
}

pub(crate) mod constants {
    /// Start address of the system's DRAM (low range).
    pub const SOC_DRAM_LOW: usize = 0x4000_0000;
    /// End address of the system's DRAM (high range).
    pub const SOC_DRAM_HIGH: usize = 0x4C00_0000;
}

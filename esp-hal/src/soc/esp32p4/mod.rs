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

/// The name of the chip ("esp32p4") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32p4"
    };
}

/// A link to the Technical Reference Manual (TRM) for the chip.
#[macro_export]
macro_rules! trm_link {
    () => {
        "https://www.espressif.com/en/products/socs/esp32-p4"
    }; // FIXME: no public TRM for now
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x500D_6000;
    pub const INTERRUPT_MAP_BASE_APP_CPU: u32 = 0x500D_6800;
}

pub(crate) mod constants {
    /// Start address of the system's DRAM (low range).
    pub const SOC_DRAM_LOW: usize = 0x4FF0_0000;
    /// End address of the system's DRAM (high range).
    pub const SOC_DRAM_HIGH: usize = 0x4FFB_FFFF;
    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

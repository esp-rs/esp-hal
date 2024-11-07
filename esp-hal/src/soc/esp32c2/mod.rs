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
pub mod trng;

/// The name of the chip ("esp32c2") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c2"
    };
}

/// A link to the Technical Reference Manual (TRM) for the chip.
#[macro_export]
macro_rules! trm_link {
    () => { "https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf" };
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

pub(crate) mod constants {
    /// The lower bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_LOW: usize = 0x3FCA_0000;
    /// The upper bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_HIGH: usize = 0x3FCE_0000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

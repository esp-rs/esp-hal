//! # SOC (System-on-Chip) module (ESP32-C3)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C3` chip.
//!
//! Also few constants are defined in this module for `ESP32-C3` chip:
//!    * I2S_SCLK: 160_000_000 - I2S clock frequency
//!    * I2S_DEFAULT_CLK_SRC: 2 - I2S clock source

pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;
pub mod trng;

/// The name of the chip ("esp32c3") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c3"
    };
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

pub(crate) mod constants {
    /// The base clock frequency for the I2S peripheral (Hertz).
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for I2S operations.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    /// The starting address of the Remote Control (RMT) module's RAM.
    pub const RMT_RAM_START: usize = 0x60016400;
    /// The size, in bytes, of each RMT channel's dedicated RAM.
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    /// RMT Clock source value.
    pub const RMT_CLOCK_SRC: u8 = 1;
    /// RMT Clock source frequence.
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(80);

    /// The lower bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_LOW: u32 = 0x3FC8_0000;
    /// The upper bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_HIGH: u32 = 0x3FCE_0000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

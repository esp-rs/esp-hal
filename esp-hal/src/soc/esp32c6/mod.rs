//! # SOC (System-on-Chip) module (ESP32-C6)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C6` chip.
//!
//! Also few constants are defined in this module for `ESP32-C6` chip:
//!    * TIMG_DEFAULT_CLK_SRC: 1 - Timer clock source
//!    * I2S_DEFAULT_CLK_SRC: 2 - I2S clock source
//!    * I2S_SCLK: 160_000_000 - I2S clock frequency

pub mod efuse;
pub mod gpio;
pub mod lp_core;
pub mod peripherals;
pub mod radio_clocks;
pub mod trng;

/// The name of the chip ("esp32c6") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c6"
    };
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    /// The default clock source for the timer group.
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 1;

    /// The clock frequency for the I2S peripheral in Hertz.
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for the I2S peripheral.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    /// The starting address of the RMT (Remote Control) peripheral's RAM.
    pub const RMT_RAM_START: usize = 0x60006400;
    /// The size of each RMT channel's RAM in bytes.
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    /// The default clock source for the RMT peripheral.
    pub const RMT_CLOCK_SRC: u8 = 1;
    /// The frequency of the RMT clock source in Hertz.
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(80);

    /// The clock frequency for the Parallel IO peripheral in Hertz.
    pub const PARL_IO_SCLK: u32 = 240_000_000;

    /// The lower address boundary for system DRAM.
    pub const SOC_DRAM_LOW: usize = 0x4080_0000;
    /// The upper address boundary for system DRAM.
    pub const SOC_DRAM_HIGH: usize = 0x4088_0000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17_500);
}

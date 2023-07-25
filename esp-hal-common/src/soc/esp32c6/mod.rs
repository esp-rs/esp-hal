//! SOC (System-on-Chip) module (ESP32-C6)
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

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 1;

    pub const I2S_SCLK: u32 = 160_000_000;
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    pub const RMT_RAM_START: usize = 0x60006400;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    pub const RMT_CLOCK_SRC: u8 = 1;
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(80);
}

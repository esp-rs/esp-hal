//! SOC (System-on-Chip) module(ESP32-S3)
//! 
//! ## Overview
//! 
//! The `SOC` module provides access, functions and structures that are useful for interacting with various
//! system-related peripherals on `ESP32-S3` chip.
//! 
//! Also few constants are defined in this module for `ESP32-S3` chip:
//!    * I2S_SCLK: 160_000_000 - I2S clock frequency
//!    * I2S_DEFAULT_CLK_SRC: 2 - I2S clock source
//! 
pub mod cpu_control;
pub mod efuse;
pub mod gpio;
pub mod peripherals;
#[cfg(psram)]
pub mod psram;
pub mod radio_clocks;
pub mod ulp_core;

pub(crate) mod constants {
    pub const I2S_SCLK: u32 = 160_000_000;
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    pub const RMT_RAM_START: usize = 0x60016800;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    pub const RMT_CLOCK_SRC: u8 = 1;
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(80);
}

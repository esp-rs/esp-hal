//! SOC (System-on-Chip) module(ESP32)
//! 
//! ## Overview
//! 
//! The `SOC` module provides access, functions and structures that are useful for interacting with various
//! system-related peripherals on `ESP32` chip.


pub mod cpu_control;
pub mod efuse;
pub mod gpio;
pub mod peripherals;
#[cfg(psram)]
pub mod psram;
pub mod radio_clocks;

pub(crate) mod constants {
    pub const I2S_SCLK: u32 = 160_000_000;
    pub const I2S_DEFAULT_CLK_SRC: u32 = 2;

    pub const RMT_RAM_START: usize = 0x3ff56800;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 64;
}

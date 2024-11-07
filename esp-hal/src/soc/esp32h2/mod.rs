//! # SOC (System-on-Chip) module (ESP32-H2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-H2` chip.
//!
//! Also few constants are defined in this module for `ESP32-H2` chip:
//!    * TIMG_DEFAULT_CLK_SRC: 2 - Timer clock source
//!    * I2S_DEFAULT_CLK_SRC: 1 - I2S clock source
//!    * I2S_SCLK: 96_000_000 - I2S clock frequency

pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;
pub mod trng;

/// The name of the chip ("esp32h2") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32h2"
    };
}

/// A link to the Technical Reference Manual (TRM) for the chip.
#[macro_export]
macro_rules! trm_link {
    () => { "https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf" };
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    /// Default clock source for the timer group (TIMG) peripheral.
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 2;

    /// Default clock source for the I2S peripheral.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 1;
    /// Clock frequency for the I2S peripheral, in Hertz.
    pub const I2S_SCLK: u32 = 96_000_000;

    /// Start address of the RMT (Remote Control) peripheral's RAM.
    pub const RMT_RAM_START: usize = 0x60007400;
    /// Size of the RAM allocated per RMT channel, in bytes.
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    /// Clock source for the RMT peripheral (false = default source).
    pub const RMT_CLOCK_SRC: bool = false;
    /// Frequency of the RMT clock source, in Hertz.
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(32);

    /// System clock frequency for the parallel I/O (PARL IO) peripheral, in
    /// Hertz.
    pub const PARL_IO_SCLK: u32 = 96_000_000;

    /// Start address of the system's DRAM (low range).
    pub const SOC_DRAM_LOW: usize = 0x4080_0000;
    /// End address of the system's DRAM (high range).
    pub const SOC_DRAM_HIGH: usize = 0x4085_0000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

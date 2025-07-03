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

crate::unstable_module! {
    pub mod efuse;
    pub mod trng;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32c3 as pac;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {
    use crate::time::Rate;

    /// The base clock frequency for the I2S peripheral (Hertz).
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for I2S operations.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    /// RMT Clock source value.
    pub const RMT_CLOCK_SRC: u8 = 1;
    /// RMT Clock source frequency.
    pub const RMT_CLOCK_SRC_FREQ: Rate = Rate::from_mhz(80);

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: Rate = Rate::from_khz(17500);
}

pub(crate) fn pre_init() {}

//! # SOC (System-on-Chip) module (ESP32-C2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C2` chip.

crate::unstable_module! {
    pub mod efuse;
    pub mod trng;
}
pub mod gpio;
pub mod peripherals;
pub(crate) mod regi2c;

/// The name of the chip ("esp32c2") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c2"
    };
}

/// A link to the Technical Reference Manual (TRM) for the chip.
#[doc(hidden)]
#[macro_export]
macro_rules! trm_link {
    () => { "https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf" };
}

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {
    use crate::time::Rate;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: Rate = Rate::from_khz(17500);
}

pub(crate) fn pre_init() {}

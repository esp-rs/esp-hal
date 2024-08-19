//! # SOC (System-on-Chip) module (ESP32-C2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C2` chip.

use self::peripherals::{LPWR, TIMG0};
use crate::{rtc_cntl::Rtc, timer::timg::Wdt};

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

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

pub(crate) mod constants {
    /// The lower bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_LOW: u32 = 0x3FCA_0000;
    /// The upper bound of the system's DRAM (Data RAM) address space.
    pub const SOC_DRAM_HIGH: u32 = 0x3FCE_0000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(LPWR::steal());
    rtc.swd.disable();
    rtc.rwdt.disable();

    Wdt::<TIMG0, crate::Blocking>::set_wdt_enabled(false);
}

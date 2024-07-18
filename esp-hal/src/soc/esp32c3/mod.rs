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

use self::peripherals::{LPWR, TIMG0, TIMG1};
use crate::{rtc_cntl::Rtc, timer::timg::Wdt};

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
    pub const I2S_SCLK: u32 = 160_000_000;
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    pub const RMT_RAM_START: usize = 0x60016400;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    pub const RMT_CLOCK_SRC: u8 = 1;
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(80);

    pub const SOC_DRAM_LOW: u32 = 0x3FC8_0000;
    pub const SOC_DRAM_HIGH: u32 = 0x3FCE_0000;

    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(LPWR::steal());
    rtc.swd.disable();
    rtc.rwdt.disable();

    Wdt::<TIMG0, crate::Blocking>::set_wdt_enabled(false);
    Wdt::<TIMG1, crate::Blocking>::set_wdt_enabled(false);
}

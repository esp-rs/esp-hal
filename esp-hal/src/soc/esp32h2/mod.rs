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

use self::peripherals::{LPWR, TIMG0, TIMG1};
use crate::{rtc_cntl::Rtc, timer::Wdt};

pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;
pub mod trng;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 2;

    pub const I2S_DEFAULT_CLK_SRC: u8 = 1;
    pub const I2S_SCLK: u32 = 96_000_000;

    pub const RMT_RAM_START: usize = 0x60007400;
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    pub const RMT_CLOCK_SRC: bool = false;
    pub const RMT_CLOCK_SRC_FREQ: fugit::HertzU32 = fugit::HertzU32::MHz(32);

    pub const PARL_IO_SCLK: u32 = 96_000_000;

    pub const SOC_DRAM_LOW: u32 = 0x4080_0000;
    pub const SOC_DRAM_HIGH: u32 = 0x4085_0000;

    pub const RC_FAST_CLK: fugit::HertzU32 = fugit::HertzU32::kHz(17500);
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(LPWR::steal(), None);
    rtc.swd.disable();
    rtc.rwdt.disable();

    Wdt::<TIMG0, crate::Blocking>::set_wdt_enabled(false);
    Wdt::<TIMG1, crate::Blocking>::set_wdt_enabled(false);
}

//! # SOC (System-on-Chip) module (ESP32-C2)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C2` chip.

use self::peripherals::{RTC_CNTL, TIMG0};
use crate::{timer::Wdt, Rtc};

pub mod efuse;
pub mod gpio;
pub mod peripherals;
pub mod radio_clocks;

pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x600c2000;
}

pub(crate) mod constants {
    pub const SOC_DRAM_LOW: u32 = 0x3FCA_0000;
    pub const SOC_DRAM_HIGH: u32 = 0x3FCE_0000;
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(RTC_CNTL::steal());
    rtc.swd.disable();
    rtc.rwdt.disable();

    Wdt::<TIMG0>::set_wdt_enabled(false);
}

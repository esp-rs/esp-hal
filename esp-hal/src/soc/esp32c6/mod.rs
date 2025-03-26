//! # SOC (System-on-Chip) module (ESP32-C6)
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

crate::unstable_module! {
    pub mod efuse;
    pub mod lp_core;
    pub mod trng;
}
pub mod gpio;
pub mod peripherals;
pub(crate) mod regi2c;

/// The name of the chip ("esp32c6") as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c6"
    };
}

/// A link to the Technical Reference Manual (TRM) for the chip.
#[doc(hidden)]
#[macro_export]
macro_rules! trm_link {
    () => { "https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf" };
}

pub use chip;

#[allow(unused)]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

pub(crate) mod constants {
    use crate::time::Rate;

    /// The default clock source for the timer group.
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 1;

    /// The clock frequency for the I2S peripheral in Hertz.
    pub const I2S_SCLK: u32 = 160_000_000;
    /// The default clock source for the I2S peripheral.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 2;

    /// The starting address of the RMT (Remote Control) peripheral's RAM.
    pub const RMT_RAM_START: usize = 0x60006400;
    /// The size (number of pulse codes) of each RMT channel's dedicated RAM.
    pub const RMT_CHANNEL_RAM_SIZE: usize = 48;
    /// The default clock source for the RMT peripheral.
    pub const RMT_CLOCK_SRC: u8 = 1;
    /// The frequency of the RMT clock source in Hertz.
    pub const RMT_CLOCK_SRC_FREQ: Rate = Rate::from_mhz(80);

    /// The clock frequency for the Parallel IO peripheral in Hertz.
    pub const PARL_IO_SCLK: u32 = 240_000_000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: Rate = Rate::from_khz(17_500);
}

pub(crate) fn pre_init() {
    // By default, these access path filters are enable and allow the access to
    // masters only if they are in TEE mode.
    //
    // Since all masters except HP CPU boot in REE mode, default setting of these
    // filters will deny the access by all masters except HP CPU.
    //
    // So, disabling these filters early.

    crate::peripherals::LP_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0x0) });
    crate::peripherals::LP_APM0::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0x0) });
    crate::peripherals::HP_APM::regs()
        .func_ctrl()
        .write(|w| unsafe { w.bits(0x0) });
}

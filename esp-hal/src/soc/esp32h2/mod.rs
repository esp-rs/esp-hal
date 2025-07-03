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

crate::unstable_module! {
    pub mod efuse;
    pub mod trng;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32h2 as pac;

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod registers {
    pub const INTERRUPT_MAP_BASE: u32 = 0x60010000;
}

#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) mod constants {
    use crate::time::Rate;

    /// Default clock source for the timer group (TIMG) peripheral.
    pub const TIMG_DEFAULT_CLK_SRC: u8 = 2;

    /// Default clock source for the I2S peripheral.
    pub const I2S_DEFAULT_CLK_SRC: u8 = 1;
    /// Clock frequency for the I2S peripheral, in Hertz.
    pub const I2S_SCLK: u32 = 96_000_000;

    /// Clock source for the RMT peripheral (false = default source).
    pub const RMT_CLOCK_SRC: bool = false;
    /// Frequency of the RMT clock source, in Hertz.
    pub const RMT_CLOCK_SRC_FREQ: Rate = Rate::from_mhz(32);

    /// System clock frequency for the parallel I/O (PARL IO) peripheral, in
    /// Hertz.
    pub const PARL_IO_SCLK: u32 = 96_000_000;

    /// RC FAST Clock value (Hertz).
    pub const RC_FAST_CLK: Rate = Rate::from_khz(17500);
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

//! # SOC (System-on-Chip) module (ESP32-C61)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C61` chip.

crate::unstable_module! {
    pub mod clocks;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32c61 as pac;

#[cfg(i2s_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn i2s_sclk_frequency() -> u32 {
    clocks::pll_f160m_frequency()
}

pub(crate) fn pre_init() {
    // Reset TEE security modes. This allows unrestricted access to TEE masters, including DMA.
    // FIXME: this is a temporary workaround until we have a proper solution for TEE security modes.
    for m in crate::peripherals::TEE::regs().m_mode_ctrl_iter() {
        m.reset();
    }

    // this is hacky, but for some reason we must reset the output enable register manually
    crate::peripherals::GPIO::regs().enable().reset();
}

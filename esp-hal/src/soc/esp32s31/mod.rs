//! # SOC (System-on-Chip) module (ESP32-C5)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C5` chip.

crate::unstable_module! {
    pub mod clocks;
}
pub mod gpio;
pub(crate) mod regi2c;

pub(crate) use esp32s31 as pac;

#[cfg(i2s_driver_supported)]
#[cfg_attr(not(feature = "unstable"), allow(unused))]
pub(crate) fn i2s_sclk_frequency() -> u32 {
    clocks::pll_f160m_frequency()
}

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    // // Reset TEE security modes. This allows unrestricted access to TEE masters, including DMA.
    // // FIXME: this is a temporary workaround until we have a proper solution for TEE security modes.
    // for m in crate::peripherals::TEE::regs().m_mode_ctrl_iter() {
    //     m.reset();
    // }
    // // this is hacky, but for some reason we must reset the output enable register manually
    // crate::peripherals::GPIO::regs().enable().reset();

    // // Clear bit reset_event_bypass to ensure that the system bus is also reset during a core reset
    // // (WDT), preventing bus freezing caused by an incorrect MSPI core reset in ROM. Mirrors
    // // ESP-IDF's bootloader_hardware_init.
    // crate::peripherals::PCR::regs()
    //     .reset_event_bypass()
    //     .modify(|_, w| w.reset_event_bypass().clear_bit());
}

//! Interrupt handling for ESP32-C6
// use core::ptr::NonNull;
// use super::{Interrupt, InterruptStatus, bound_handler};
use super::Interrupt;

/// Setup interrupt handlers, including any default ones
#[inline(always)]
pub fn setup_interrupts() {}

/// Enables or disables a peripheral interrupt.
///
/// Note that interrupts still need to be enabled globally for interrupts
/// to be serviced.
///
/// Internally, this function maps the interrupt to the appropriate CPU interrupt.
#[inline]
pub fn set_enabled(_interrupt: Interrupt, _enable: bool) {
    // todo!()
}

/// Returns a bitmask of active interrupts
pub fn current_interrupts() -> u32 {
    todo!()
}

/// Set the IRQ Mask, returns the previous mask value.
#[inline(always)]
pub fn mask_cpu_interrupts(_new_mask: u32) -> u32 {
    todo!()
}

/// Enable all CPU interrupts by setting IRQ mask
#[inline(always)]
pub fn enable_cpu_interrupts() {
    // mask_cpu_interrupts(0x0);
    todo!()
}

/// Disable all CPU interrupts by clearing IRQ mask,
/// returns the previous IRQ Mask
#[inline(always)]
pub fn disable_cpu_interrupts() -> u32 {
    // mask_cpu_interrupts(0xFFFF);
    todo!()
}

/// Default interrupt handler, does nothing.
#[allow(dead_code)]
#[doc(hidden)]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub fn DefaultHandler() {}

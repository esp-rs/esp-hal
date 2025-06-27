//! # Peripheral Instances
//!
//! This module creates singleton instances for each of the various peripherals,
//! and re-exports them to allow users to access and use them in their
//! applications.
//!
//! Should be noted that that the module also re-exports the [Interrupt] enum
//! from the PAC, allowing users to handle interrupts associated with these
//! peripherals.

pub(crate) use esp32c3 as pac;
// We need to export this for users to use
#[doc(hidden)]
pub use pac::Interrupt;

include!(concat!(env!("OUT_DIR"), "/_generated_peris.rs"));
include!(concat!(env!("OUT_DIR"), "/_generated_gpio.rs"));

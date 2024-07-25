//! # Analog Peripherals
//!
//! ## Overview
//! The `analog` module provides drivers for the various analog peripherals
//! available on the device. For more information about a peripheral driver,
//! please refer to the relevant module documentation.

#![deny(missing_docs)]

#[cfg(adc)]
pub mod adc;
#[cfg(dac)]
pub mod dac;

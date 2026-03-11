//! # Analog Peripherals
//!
//! ## Overview
//! The `analog` module provides drivers for the various analog peripherals
//! available on the device. For more information about a peripheral driver,
//! please refer to the relevant module documentation.

#[cfg(adc_driver_supported)]
pub mod adc;
#[cfg(dac_driver_supported)]
pub mod dac;

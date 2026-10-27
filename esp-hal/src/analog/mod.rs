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

// DAC channels share SENS control registers. On ESP32-S2, ADC initialization
// also modifies SAR_AMP_CTRL3.
#[cfg(dac_driver_supported)]
static ADC_DAC_LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

//! Digital to analog (DAC) conversion.
//!
//! This module provides functions for controling two digital to
//! analog converters, available on ESP32: `DAC1` and `DAC2`.
//!
//! The DAC1 is available on the GPIO pin 25, and DAC2 on pin 26.

pub use esp_hal_common::analog::dac::*;
use esp_hal_common::{impl_dac, paste};

impl_dac!(1 => Gpio25, 2 => Gpio26,);

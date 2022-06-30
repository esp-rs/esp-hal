//! Digital to analog (DAC) conversion.
//!
//! This module provides functions for controling two digital to
//! analog converters, available on ESP32-S2: `DAC1` and `DAC2`.
//!
//! The DAC1 is available on the GPIO pin 17, and DAC2 on pin 18.

use esp_hal_common::{impl_dac, paste};

impl_dac!(1 => Gpio17, 2 => Gpio18,);

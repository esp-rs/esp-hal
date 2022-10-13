//! Analog to digital (ADC) conversion support.
//!
//! This module provides functions for reading analog values from the
//! analog to digital converter available on the ESP32-C2: `ADC1`.

use embedded_hal::adc::Channel;
use esp_hal_common::analog::adc::impl_adc_interface;
pub use esp_hal_common::analog::{adc::*, ADC1};

use crate::{gpio::*, gpio_types::Analog};

impl_adc_interface! {
    ADC1 [
        (Gpio0, 0),
        (Gpio1, 1),
        (Gpio2, 2),
        (Gpio3, 3),
        (Gpio4, 4),
    ]
}

//! Analog to digital (ADC) conversion support.
//!
//! This module provides functions for reading analog values from two
//! analog to digital converters available on the ESP32-S2: `ADC1` and `ADC2`.

use embedded_hal::adc::Channel;
use esp_hal_common::analog::adc::impl_adc_interface;
pub use esp_hal_common::analog::{adc::*, ADC1, ADC2};

use crate::{gpio::*, gpio_types::Analog};

impl_adc_interface! {
    ADC1 [
        (Gpio1, 0),
        (Gpio2, 1),
        (Gpio3, 2),
        (Gpio4, 3),
        (Gpio5, 4),
        (Gpio6, 5),
        (Gpio7, 6),
        (Gpio8, 7),
        (Gpio9, 8),
        (Gpio10,9),
    ]
}

impl_adc_interface! {
    ADC2 [
        (Gpio11, 0),
        (Gpio12, 1),
        (Gpio13, 2),
        (Gpio14, 3),
        (Gpio15, 4),
        (Gpio16, 5),
        (Gpio17, 6),
        (Gpio18, 7),
        (Gpio19, 8),
        (Gpio20, 9),
    ]
}

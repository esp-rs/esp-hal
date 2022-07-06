//! Analog to digital (ADC) conversion support.
//!
//! This module provides functions for reading analog values from two
//! analog to digital converters available on the ESP32: `ADC1` and `ADC2`.
//!
//! The following pins can be configured for analog readout:
//!
//! | Channel | ADC1                 | ADC2          |
//! |---------|----------------------|---------------|
//! | 0       | GPIO36 (SENSOR_VP)   | GPIO4         |
//! | 1       | GPIO37 (SENSOR_CAPP) | GPIO0         |
//! | 2       | GPIO38 (SENSOR_CAPN) | GPIO2         |
//! | 3       | GPIO39 (SENSOR_VN)   | GPIO15 (MTDO) |
//! | 4       | GPIO33 (32K_XP)      | GPIO13 (MTCK) |
//! | 5       | GPIO32 (32K_XN)      | GPIO12 (MTDI) |
//! | 6       | GPIO34 (VDET_1)      | GPIO14 (MTMS) |
//! | 7       | GPIO35 (VDET_2)      | GPIO27        |
//! | 8       |                      | GPIO25        |
//! | 9       |                      | GPIO26        |

use embedded_hal::adc::Channel;
use esp_hal_common::analog::{adc::impl_adc_interface, ADC1, ADC2};

use crate::{gpio::*, gpio_types::Analog};

impl_adc_interface! {
    ADC1 [
        (Gpio36, 0), // Alt. name: SENSOR_VP
        (Gpio37, 1), // Alt. name: SENSOR_CAPP
        (Gpio38, 2), // Alt. name: SENSOR_CAPN
        (Gpio39, 3), // Alt. name: SENSOR_VN
        (Gpio33, 4), // Alt. name: 32K_XP
        (Gpio32, 5), // Alt. name: 32K_XN
        (Gpio34, 6), // Alt. name: VDET_1
        (Gpio35, 7), // Alt. name: VDET_2
    ]
}

impl_adc_interface! {
    ADC2 [
        (Gpio4, 0),
        (Gpio0, 1),
        (Gpio2, 2),
        (Gpio15, 3), // Alt. name: MTDO
        (Gpio13, 4), // Alt. name: MTCK
        (Gpio12, 5), // Alt. name: MTDI
        (Gpio14, 6), // Alt. name: MTMS
        (Gpio27, 7),
        (Gpio25, 8),
        (Gpio26, 9),
    ]
}

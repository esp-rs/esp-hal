//! # Pixel Processing Accelerator (PPA)
//!
//! ## Overview
//! The PPA performs image transformations and color conversion.
//!
//! For more information, see
#![doc = concat!(
    "[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/",
    chip!(),
    "/api-reference/peripherals/ppa.html)"
)]
//! ## Implementation State
//! Only the ESP32-S31 [`srm`] driver is implemented.

pub mod srm;

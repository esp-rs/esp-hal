//! `no_std` HAL for the ESP32-H2 from Espressif.
//!
//! Implements a number of the traits defined by the various packages in the
//! [embedded-hal] repository.
//!
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal

#![no_std]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

pub use esp_hal_common::*;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

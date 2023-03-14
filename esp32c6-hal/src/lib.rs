#![no_std]

pub use embedded_hal as ehal;
#[cfg(feature = "embassy")]
pub use esp_hal_common::embassy;
pub use esp_hal_common::*;

pub use self::gpio::IO;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

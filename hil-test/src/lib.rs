#![no_std]

use defmt_rtt as _;
use panic_probe as _;

pub mod esp_hal {
    #[cfg(feature = "esp32")]
    pub use esp32_hal::*;
    #[cfg(feature = "esp32c2")]
    pub use esp32c2_hal::*;
    #[cfg(feature = "esp32c3")]
    pub use esp32c3_hal::*;
    #[cfg(feature = "esp32c6")]
    pub use esp32c6_hal::*;
    #[cfg(feature = "esp32h2")]
    pub use esp32h2_hal::*;
    #[cfg(feature = "esp32s2")]
    pub use esp32s2_hal::*;
    #[cfg(feature = "esp32s3")]
    pub use esp32s3_hal::*;
}

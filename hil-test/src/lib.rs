#![no_std]

use defmt_rtt as _;
use panic_probe as _;

pub mod esp_hal {
    pub use esp_hal::*;
}

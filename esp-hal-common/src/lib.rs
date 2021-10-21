#![no_std]

#[cfg(feature = "esp32")]
pub use esp32 as pac;
#[cfg(feature = "esp32c3")]
pub use esp32c3 as pac;
#[cfg(feature = "esp32s2")]
pub use esp32s2 as pac;

pub mod timer;

pub use timer::Timer;

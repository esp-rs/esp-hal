#![no_std]

#[cfg(feature = "esp32")]
pub use esp32_pac as pac;
#[cfg(feature = "esp32c3")]
pub use esp32c3_pac as pac;
#[cfg(feature = "esp32s2")]
pub use esp32s2_pac as pac;
#[cfg(feature = "esp32s3")]
pub use esp32s3_pac as pac;

pub mod gpio;
pub mod prelude;
pub mod serial;
pub mod timer;

pub use gpio::*;
pub use serial::Serial;
pub use timer::Timer;

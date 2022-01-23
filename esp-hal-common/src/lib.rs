//! `no_std` HAL implementations for the peripherals which are common among
//! Espressif devices. Implements a number of the traits defined by
//! [embedded-hal].
//!
//! This crate should not be used directly; you should use one of the
//! device-specific HAL crates instead:
//!
//! - [esp32-hal]
//! - [esp32c3-hal]
//! - [esp32s2-hal]
//! - [esp32s3-hal]
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
//! [esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
//! [esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
//! [esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal

#![no_std]

#[cfg(feature = "esp32")]
pub use esp32_pac as pac;
#[cfg(feature = "esp32c3")]
pub use esp32c3_pac as pac;
#[cfg(feature = "esp32s2")]
pub use esp32s2_pac as pac;
#[cfg(feature = "esp32s3")]
pub use esp32s3_pac as pac;

pub mod delay;
pub mod gpio;
#[cfg_attr(feature = "esp32", path = "interrupt/xtensa.rs")]
#[cfg_attr(feature = "esp32c3", path = "interrupt/riscv.rs")]
pub mod interrupt;
pub mod prelude;
pub mod serial;
pub mod timer;

pub use delay::Delay;
pub use gpio::*;
pub use interrupt::*;
use procmacros;
pub use procmacros::ram;
pub use serial::Serial;
pub use timer::Timer;

/// Enumeration of CPU cores
/// The actual number of available cores depends on the target.
pub enum Cpu {
    /// The fist core
    ProCpu = 0,
    /// The second core
    AppCpu,
}

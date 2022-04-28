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
pub mod i2c;
#[cfg_attr(feature = "risc_v", path = "interrupt/riscv.rs")]
#[cfg_attr(feature = "xtensa", path = "interrupt/xtensa.rs")]
pub mod interrupt;
pub mod prelude;
pub mod rng;
#[cfg(not(feature = "esp32c3"))]
pub mod rtc_cntl;
pub mod serial;
pub mod spi;
pub mod timer;
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
pub mod usb_serial_jtag;

pub use delay::Delay;
pub use gpio::*;
pub use interrupt::*;
use procmacros;
pub use procmacros::ram;
pub use rng::Rng;
#[cfg(not(feature = "esp32c3"))]
pub use rtc_cntl::RtcCntl;
pub use serial::Serial;
pub use spi::Spi;
pub use timer::Timer;
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
pub use usb_serial_jtag::UsbSerialJtag;

/// Enumeration of CPU cores
/// The actual number of available cores depends on the target.
pub enum Cpu {
    /// The fist core
    ProCpu = 0,
    /// The second core
    AppCpu,
}

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

#[cfg_attr(feature = "esp32", path = "efuse/esp32.rs")]
#[cfg_attr(feature = "esp32c3", path = "efuse/esp32c3.rs")]
#[cfg_attr(feature = "esp32s2", path = "efuse/esp32s2.rs")]
#[cfg_attr(feature = "esp32s3", path = "efuse/esp32s3.rs")]
pub mod efuse;

pub mod gpio;
pub mod i2c;
#[cfg_attr(target_arch = "riscv32", path = "interrupt/riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "interrupt/xtensa.rs")]
pub mod interrupt;
pub mod ledc;
pub mod prelude;
pub mod pulse_control;
pub mod rng;
pub mod rom;
pub mod rtc_cntl;
pub mod serial;
pub mod spi;
pub mod timer;
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
pub mod usb_serial_jtag;
pub mod utils;

pub use delay::Delay;
pub use gpio::*;
pub use interrupt::*;
pub use procmacros as macros;
pub use pulse_control::PulseControl;
pub use rng::Rng;
pub use rtc_cntl::RtcCntl;
pub use serial::Serial;
pub use spi::Spi;
pub use timer::Timer;
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
pub use usb_serial_jtag::UsbSerialJtag;
#[cfg(any(feature = "esp32c3", feature = "esp32s3", feature = "esp32s2"))]
pub mod systimer;

pub mod clock;
pub mod system;

pub mod analog;

#[cfg_attr(feature = "esp32", path = "cpu_control/esp32.rs")]
#[cfg_attr(feature = "esp32c3", path = "cpu_control/none.rs")]
#[cfg_attr(feature = "esp32s2", path = "cpu_control/none.rs")]
#[cfg_attr(feature = "esp32s3", path = "cpu_control/esp32s3.rs")]
pub mod cpu_control;

/// Enumeration of CPU cores
/// The actual number of available cores depends on the target.
pub enum Cpu {
    /// The first core
    ProCpu = 0,
    /// The second core
    AppCpu,
}

pub fn get_core() -> Cpu {
    #[cfg(all(target_arch = "xtensa", feature = "multi_core"))]
    match ((xtensa_lx::get_processor_id() >> 13) & 1) != 0 {
        false => Cpu::ProCpu,
        true => Cpu::AppCpu,
    }
    // #[cfg(all(target_arch = "riscv32", feature = "multi_core"))]
    // TODO get hart_id

    // single core always has ProCpu only
    #[cfg(feature = "single_core")]
    Cpu::ProCpu
}

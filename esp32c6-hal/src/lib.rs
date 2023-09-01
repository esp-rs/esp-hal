//! `no_std` HAL for the ESP32-C6 from Espressif.
//!
//! Implements a number of the traits defined by the various packages in the
//! [embedded-hal] repository.
//!
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal
//!
//! ### Cargo Features
//!
//! - `async` - Enable support for asynchronous operation, with interfaces
//!   provided by [embedded-hal-async] and [embedded-io-async]
//! - `debug` - Enable debug features in the HAL (used for development)
//! - `defmt` - Enable [`defmt::Format`] on certain types
//! - `direct-boot` - Use the direct boot image format
//! - `direct-vectoring` - Enable direct vector table hooking support
//! - `eh1` - Implement the traits defined in the `1.0.0-xxx` pre-releases of
//!   [embedded-hal], [embedded-hal-nb], and [embedded-io]
//! - `embassy` - Enable support for [embassy], a modern asynchronous embedded
//!   framework
//! - `embassy-time-systick` - Enable the [embassy] time driver using the
//!   `SYSTIMER` peripheral
//! - `embassy-time-timg0` - Enable the [embassy] time driver using the `TIMG0`
//!   peripheral
//! - `interrupt-preemption` - Enable priority-based interrupt preemption
//! - `log` - enable log output using the `log` crate
//! - `rt` - Runtime support
//! - `ufmt` - Implement the [`ufmt_write::uWrite`] trait for the UART and USB
//!   Serial JTAG drivers
//! - `vectored` - Enable interrupt vectoring
//!
//! #### Default Features
//!
//! The `rt` and `vectored` features are enabled by default.
//!
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [embedded-io-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io-async
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-nb]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-nb
//! [embedded-io]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io
//! [embassy]: https://github.com/embassy-rs/embassy
//! [`ufmt_write::uWrite`]: https://docs.rs/ufmt-write/latest/ufmt_write/trait.uWrite.html
//! [`defmt::Format`]: https://docs.rs/defmt/0.3.5/defmt/trait.Format.html
//!
//! ### Supported Image Formats
//!
//! This HAL supports building multiple different application image formats. You
//! can read about each below.
//!
//! The ESP-IDF Bootloader format is used unless some other format is specified
//! via its feature.
//!
//! #### ESP-IDF Bootloader
//!
//! Use the second-stage bootloader from [ESP-IDF] and its associated
//! application image format. See the [App Image Format] documentation for more
//! information about this format.
//!
//! [ESP-IDF]: https://github.com/espressif/esp-idf
//! [App Image Format]: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/app_image_format.html
//!
//! #### Direct Boot
//!
//! This device additionally supports direct-boot, which allows an application
//! to be executed directly from flash, without using the second-stage
//! bootloader. For more information please see the
//! [esp32c3-direct-boot-example] in the Espressif organization on GitHub.
//!
//! [esp32c3-direct-boot-example]: https://github.com/espressif/esp32c3-direct-boot-example

#![no_std]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

pub use esp_hal_common::*;

/// Common module for analog functions
pub mod analog {
    pub use esp_hal_common::analog::{AvailableAnalog, SarAdcExt};
}

#[export_name = "__post_init"]
unsafe fn post_init() {
    use esp_hal_common::{
        peripherals::{LP_CLKRST, TIMG0, TIMG1},
        timer::Wdt,
    };

    // RTC domain must be enabled before we try to disable
    let mut rtc = Rtc::new(LP_CLKRST::steal());
    rtc.swd.disable();
    rtc.rwdt.disable();

    Wdt::<TIMG0>::set_wdt_enabled(false);
    Wdt::<TIMG1>::set_wdt_enabled(false);
}

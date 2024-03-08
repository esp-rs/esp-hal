//! `no_std` HAL for the ESP32-S3 from Espressif.
//!
//! <div class="warning">
//! This package has been deprecated in favour of <a href="https://github.com/esp-rs/esp-hal/tree/main/esp-hal">esp-hal</a>.
//! Please refer to the migration guide for help with updating your projects
//! to use the new <em>esp-hal</em> package:
//! <br /><br />
//! <a href="https://github.com/esp-rs/esp-hal/releases/tag/v0.16.0">https://github.com/esp-rs/esp-hal/releases/tag/v0.16.0</a>
//! </div>
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
//! - `eh1` - Implement the traits defined in the `1.0.0-xxx` pre-releases of
//!   [embedded-hal], [embedded-hal-nb], and [embedded-io]
//! - `embassy` - Enable support for [embassy], a modern asynchronous embedded
//!   framework. One of `embassy-time-*` features must also be enabled when
//!   using this feature.
//! - `embassy-executor-interrupt` - Use the multicore-aware interrupt-mode
//!   embassy executor
//! - `embassy-executor-thread` - Use the multicore-aware thread-mode embassy
//!   executor
//! - `embassy-time-systick` - Enable the [embassy] time driver using the
//!   `SYSTIMER` peripheral. The `SYSTIMER` peripheral has three alarms
//!   available for use
//! - `embassy-time-timg0` - Enable the [embassy] time driver using the `TIMG0`
//!   peripheral. The `TIMG0` peripheral has two alarms available for use
//! - `embassy-integrated-timers` - Uses hardware timers as alarms for the
//!   executors. Using this feature limits the number of executors to the number
//!   of hardware alarms provided by the time driver
//! - `embassy-generic-queue-N` (where `N` can be `8`, `16`, `32`, `64` or
//!   `128`) - Use a generic timer queue of size `N` for the executors' timer
//!   queues. Using this feature can expand the number of executors you can use
//!   to `N`
//! - `log` - enable log output using the `log` crate
//! - `opsram-2m` - Use externally connected Octal PSRAM (2MB)
//! - `opsram-4m` - Use externally connected Octal PSRAM (4MB)
//! - `opsram-8m` - Use externally connected Octal PSRAM (8MB)
//! - `opsram-16m`- Use externally connected Octal PSRAM (16MB)
//! - `psram-2m` - Use externally connected PSRAM (2MB)
//! - `psram-4m` - Use externally connected PSRAM (4MB)
//! - `psram-8m` - Use externally connected PSRAM (8MB)
//! - `psram-80mhz` - Use faster 80Mhz PSRAM frequencty instead of default 40Mhz
//! - `rt` - Runtime support
//! - `ufmt` - Implement the [`ufmt_write::uWrite`] trait for the UART driver
//! - `vectored` - Enable interrupt vectoring
//!
//! #### Default Features
//!
//! The `rt`, `vectored` and `embassy-integrated-timers` features are enabled by
//! default.
//!
//! [embedded-hal-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-async
//! [embedded-io-async]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io-async
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal
//! [embedded-hal-nb]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-nb
//! [embedded-io]: https://github.com/rust-embedded/embedded-hal/tree/master/embedded-io
//! [embassy]: https://github.com/embassy-rs/embassy
//! [`ufmt_write::uWrite`]: https://docs.rs/ufmt-write/latest/ufmt_write/trait.uWrite.html
//! [`defmt::Format`]: https://docs.rs/defmt/0.3.5/defmt/trait.Format.html
#![no_std]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

pub use esp_hal_common::*;

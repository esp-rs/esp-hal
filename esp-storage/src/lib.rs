//! `esp-storage` contains API functions related to reading, writing and erasing memory for data in
//! the external flash.
//!
//! For higher-level functionality which works with partitions defined in the partition table, see
//! `esp-bootloader-esp-idf`.
//!
//! The `embedded-storage` feature flag only implements traits from [`embedded_storage`].
//!
//! If you need to use this struct where traits from [`embedded_storage_async`] are needed, you can
//! use [`embassy_embedded_hal::adapter::BlockingAsync`] or
//! [`embassy_embedded_hal::adapter::YieldingAsync`] wrappers.
//!
//! [`embedded_storage`]: https://docs.rs/embedded-storage/latest/embedded_storage/
//! [`embedded_storage_async`]: https://docs.rs/embedded-storage-async/latest/embedded_storage_async/
//! [`embassy_embedded_hal::adapter::BlockingAsync`]: https://docs.rs/embassy-embedded-hal/latest/embassy_embedded_hal/adapter/struct.BlockingAsync.html
//! [`embassy_embedded_hal::adapter::YieldingAsync`]: https://docs.rs/embassy-embedded-hal/latest/embassy_embedded_hal/adapter/struct.YieldingAsync.html
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![cfg_attr(not(all(test, feature = "emulation")), no_std)]

#[cfg_attr(not(feature = "emulation"), path = "hardware.rs")]
#[cfg_attr(feature = "emulation", path = "stub.rs")]
mod chip_specific;

mod buffer;
mod common;

pub use common::{Flash, FlashStorage, FlashStorageError};

pub mod ll;
mod nor_flash;
mod storage;

#[cfg(not(feature = "emulation"))]
#[inline(always)]
fn maybe_with_critical_section<R>(f: impl FnOnce() -> R) -> R {
    #[cfg(feature = "critical-section")]
    {
        static LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

        LOCK.lock(f)
    }

    #[cfg(not(feature = "critical-section"))]
    f()
}

#[cfg(feature = "emulation")]
fn maybe_with_critical_section<R>(f: impl FnOnce() -> R) -> R {
    f()
}

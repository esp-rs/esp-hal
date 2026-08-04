//! `esp-storage` contains API functions related to reading, writing and erasing memory for data in
//! the external flash.
//!
//! For higher-level functionality which works with partitions defined in the partition table, see
//! `esp-bootloader-esp-idf`.
//!
//! Encrypted flash read/write is available on chips with hardware flash encryption support when
//! flash encryption is enabled in eFuses. See [`FlashStorage::read_encrypted`] and
//! [`FlashStorage::write_encrypted`].
//!
//! The `embedded-storage` feature flag only implements traits from [`embedded_storage`]
//! and always assumes access to non-encrypted flash.
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
//! ## Buffer alignment and stack usage
//!
//! The ESP flash ROM read/write path requires word-aligned (4-byte) buffers.
//! Several methods accept any `&[u8]` / `&mut [u8]` and, when needed, copy
//! through a temporary sector-sized buffer on the stack. That cost is not
//! visible from the slice type and can surprise users in stack-constrained
//! contexts (for example Embassy tasks with small stacks).
//!
//! [`FlashStorage::read_nor`] and [`FlashStorage::write_nor`] only allocate
//! this fallback when the caller's slice pointer is not word-aligned. Each
//! fallback is [`FlashStorage::SECTOR_SIZE`] bytes.
//!
//! [`FlashStorage::read`], [`FlashStorage::write`], and the encrypted variants
//! always allocate a sector buffer on the stack for each call.
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![cfg_attr(not(all(test, feature = "emulation")), no_std)]

#[macro_use]
extern crate esp_metadata_generated;

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
mod mmu;

mod encrypted;

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

/// Whether flash encryption is enabled or not.
#[cfg(not(feature = "emulation"))]
pub fn flash_encryption() -> bool {
    esp_hal::efuse::flash_encryption()
}

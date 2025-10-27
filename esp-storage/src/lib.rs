//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![cfg_attr(not(all(test, feature = "emulation")), no_std)]

#[cfg_attr(not(feature = "emulation"), path = "hardware.rs")]
#[cfg_attr(feature = "emulation", path = "stub.rs")]
mod chip_specific;

mod buffer;
mod common;

pub use common::{FlashStorage, FlashStorageError};

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

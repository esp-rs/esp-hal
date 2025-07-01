//! ## Feature Flags
#![doc = document_features::document_features!()]
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
#[cfg_attr(not(target_os = "macos"), unsafe(link_section = ".rwtext"))]
fn maybe_with_critical_section<R>(f: impl FnOnce() -> R) -> R {
    #[cfg(feature = "critical-section")]
    return critical_section::with(|_| f());

    #[cfg(not(feature = "critical-section"))]
    f()
}

#[cfg(feature = "emulation")]
fn maybe_with_critical_section<R>(f: impl FnOnce() -> R) -> R {
    f()
}

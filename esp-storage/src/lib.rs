//! ## Feature Flags
#![doc = document_features::document_features!()]
#![cfg_attr(not(all(test, feature = "emulation")), no_std)]

#[cfg_attr(feature = "esp32c2", path = "esp32c2.rs")]
#[cfg_attr(feature = "esp32c3", path = "esp32c3.rs")]
#[cfg_attr(feature = "esp32c6", path = "esp32c6.rs")]
#[cfg_attr(feature = "esp32h2", path = "esp32h2.rs")]
#[cfg_attr(feature = "esp32", path = "esp32.rs")]
#[cfg_attr(feature = "esp32s2", path = "esp32s2.rs")]
#[cfg_attr(feature = "esp32s3", path = "esp32s3.rs")]
#[cfg_attr(feature = "emulation", path = "stub.rs")]
mod chip_specific;

mod common;

use common::FlashSectorBuffer;
pub use common::{FlashStorage, FlashStorageError};

pub mod ll;
mod nor_flash;
mod storage;

#[cfg(not(feature = "emulation"))]
#[inline(always)]
#[unsafe(link_section = ".rwtext")]
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

#[doc(hidden)]
#[macro_export]
macro_rules! rom_fn {
    ($(#[$attrs:meta])* fn $name:ident($($arg:tt: $ty:ty),*) $(-> $retval:ty)? = $addr:expr) => {
        $(#[$attrs])*
        #[allow(unused)]
        #[inline(always)]
        #[unsafe(link_section = ".rwtext")]
        fn $name($($arg:$ty),*) $(-> $retval)? {
            unsafe {
                let rom_fn: unsafe extern "C" fn($($arg: $ty),*) $(-> $retval)? =
                    core::mem::transmute($addr as usize);
                rom_fn($($arg),*)
            }
        }
    };

    ($($(#[$attrs:meta])* fn $name:ident($($arg:tt: $ty:ty),*) $(-> $retval:ty)? = $addr:expr;)+) => {
        $(
            $crate::rom_fn!(fn $name($($arg: $ty),*) $(-> $retval)? = $addr);
        )+
    };
}

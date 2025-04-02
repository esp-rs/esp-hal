//! Metadata for Espressif devices, primarily intended for use in build scripts.
#![cfg_attr(not(feature = "build"), no_std)]

#[cfg(feature = "build")]
mod generate_cfg;

#[cfg(feature = "build")]
pub use generate_cfg::*;

/// Macro to get the start of the given memory region.
///
/// ```rust,no-run
/// const DRAM_START: usize = esp_metadata::memory_region_start("DRAM");
/// ```
#[macro_export]
macro_rules! memory_region_start {
    ( $var:expr ) => {
        const {
            match usize::from_str_radix(env!(concat!("ESP_METADATA_REGION_", $var, "_START")), 10) {
                Ok(val) => val,
                Err(_) => {
                    core::assert!(false, "Unable to parse memory region.");
                    0
                }
            }
        }
    };
}

/// Macro to get the end of the given memory region.
///
/// ```rust,no-run
/// const DRAM_END: usize = esp_metadata::memory_region_end("DRAM");
/// ```
#[macro_export]
macro_rules! memory_region_end {
    ( $var:expr ) => {
        const {
            match usize::from_str_radix(env!(concat!("ESP_METADATA_REGION_", $var, "_END")), 10) {
                Ok(val) => val,
                Err(_) => {
                    core::assert!(false, "Unable to parse memory region.");
                    0
                }
            }
        }
    };
}

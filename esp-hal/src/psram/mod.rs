#![cfg_attr(docsrs, procmacros::doc_replace(
    "octal" => {
        cfg(octal_psram) => "Either `Octal` or `Quad` PSRAM will be used, depending on the setting of `ESP_HAL_CONFIG_PSRAM_MODE`.",
        _ => ""
    }
))]
//! # PSRAM (Pseudo-static RAM, SPI RAM) driver
//!
//! ## Overview
//!
//! The `PSRAM` module provides support for accessing and controlling the
//! `Pseudo Static Random Access Memory (PSRAM)`. `PSRAM` provides additional
//! external memory to supplement the internal memory of the MCU, allowing for
//! increased storage capacity and improved performance in certain applications.
//!
//! The mapped start address for PSRAM depends on the amount of mapped flash
//! memory.
//!
//! ## Examples
//!
//! ### PSRAM as heap memory
//!
//! This example shows how to use PSRAM as heap-memory via esp-alloc.
//! You need an MCU with at least 2 MB of PSRAM memory.
//! # {octal}
//!
//! > The PSRAM example **must** be built in release mode!
//!
//! ```rust, ignore
//! # {before_snippet}
//! # extern crate alloc;
//! # use alloc::{string::String, vec::Vec};
//! #
//! // Add PSRAM to the heap.
//! esp_alloc::psram_allocator!(&peripherals.PSRAM, esp_hal::psram);
//!
//! let mut large_vec: Vec<u32> = Vec::with_capacity(500 * 1024 / 4);
//!
//! for i in 0..(500 * 1024 / 4) {
//!     large_vec.push((i & 0xff) as u32);
//! }
//!
//! let string = String::from("A string allocated in PSRAM");
//! # {after_snippet}
//! ```

use core::ops::Range;

#[cfg(feature = "psram")]
#[cfg_attr(docsrs, doc(cfg(feature = "psram")))]
#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
pub(crate) mod implem;

#[cfg(feature = "psram")]
#[cfg_attr(docsrs, doc(cfg(feature = "psram")))]
pub use implem::*;

/// Size of PSRAM
///
/// [PsramSize::AutoDetect] will try to detect the size of PSRAM
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramSize {
    /// Detect PSRAM size
    #[default]
    AutoDetect,
    /// A fixed PSRAM size
    Size(usize),
}

impl PsramSize {
    #[cfg_attr(not(feature = "psram"), expect(unused))]
    pub(crate) fn get(&self) -> usize {
        match self {
            PsramSize::AutoDetect => 0,
            PsramSize::Size(size) => *size,
        }
    }

    #[cfg_attr(not(feature = "psram"), expect(unused))]
    pub(crate) fn is_auto(&self) -> bool {
        matches!(self, PsramSize::AutoDetect)
    }
}

/// Returns the address and size of the available in external memory.
#[cfg(feature = "psram")]
pub fn psram_raw_parts(_psram: &crate::peripherals::PSRAM<'_>) -> (*mut u8, usize) {
    let range = psram_range();
    (range.start as *mut u8, range.end - range.start)
}

// Using static mut should be fine since we are only writing to it once during
// initialization. As other tasks and interrupts are not running yet, the worst
// that can happen is, that the user creates a DMA buffer before initializing
// the HAL. This will access the PSRAM range, returning an empty range - which
// is, at that point, true. The user has no (safe) means to allocate in PSRAM
// before initializing the HAL.
#[cfg(feature = "psram")]
static mut MAPPED_PSRAM: MappedPsram = MappedPsram { memory_range: 0..0 };

pub(crate) fn psram_range() -> Range<usize> {
    cfg_if::cfg_if! {
        if #[cfg(feature = "psram")] {
            #[allow(static_mut_refs)]
            unsafe { MAPPED_PSRAM.memory_range.clone() }
        } else {
            0..0
        }
    }
}

#[cfg(feature = "psram")]
pub(crate) struct MappedPsram {
    memory_range: Range<usize>,
}

#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # PSRAM (Pseudo-static RAM, SPI RAM) driver
//!
//! ## Overview
//!
//! This module provides support to interface with `PSRAM` devices connected to the MCU.
//! PSRAM provides additional external memory to supplement the internal memory of the MCU,
//! allowing for increased storage capacity and improved performance in certain applications.
#![doc = ""]
#![cfg_attr(
    psram_octal_spi,
    doc = concat!("The ", chip_pretty!(), " can use either Quad SPI or Octal SPI to interface with PSRAM.
        You need to configure the correct interface type using `ESP_HAL_CONFIG_PSRAM_MODE`.")
)]
#![doc = ""]
//! ## Examples
//!
//! ### PSRAM as heap memory
//!
//! This example shows how to use PSRAM as heap-memory via esp-alloc.
//!
//! <section class="warning">
//! The PSRAM example <em>must</em> be built in release mode!
//! </section>
//!
//! ```rust, ignore
//! # {before_snippet}
//! extern crate alloc;
//! use alloc::{string::String, vec::Vec};
//!
//! // Add PSRAM to the heap.
//! esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
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

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(any(esp32c5, esp32c61), path = "esp32c5_c61.rs")]
pub(crate) mod implem;

pub use implem::*;

use crate::peripherals::PSRAM;

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
    pub(crate) fn get(&self) -> usize {
        match self {
            PsramSize::AutoDetect => 0,
            PsramSize::Size(size) => *size,
        }
    }

    pub(crate) fn is_auto(&self) -> bool {
        matches!(self, PsramSize::AutoDetect)
    }
}

// Using static mut should be fine since we are only writing to it once during
// initialization. As other tasks and interrupts are not running yet, the worst
// that can happen is, that the user creates a DMA buffer before initializing
// the HAL. This will access the PSRAM range, returning an empty range - which
// is, at that point, true. The user has no (safe) means to allocate in PSRAM
// before initializing the HAL.
static mut MAPPED_PSRAM: MappedPsram = MappedPsram { memory_range: 0..0 };

pub(crate) fn psram_range() -> Range<usize> {
    #[allow(static_mut_refs)]
    unsafe {
        MAPPED_PSRAM.memory_range.clone()
    }
}

pub(crate) struct MappedPsram {
    memory_range: Range<usize>,
}

/// Enables externally-connected Pseudo-static RAM.
pub struct Psram {
    _peri: PSRAM<'static>,
}

impl Psram {
    /// Initializes PSRAM.
    pub fn new(peri: PSRAM<'static>, config: PsramConfig) -> Self {
        init_psram(config);
        Self { _peri: peri }
    }

    /// Returns the address and size of the available in external memory.
    pub fn raw_parts(&self) -> (*mut u8, usize) {
        let range = psram_range();
        (range.start as *mut u8, range.end - range.start)
    }
}

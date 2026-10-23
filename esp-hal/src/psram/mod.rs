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
    esp32s3,
    doc = concat!("The ", chip_pretty!(), " can use either Quad SPI or Octal SPI to interface with PSRAM.
        `esp-hal` will try to automatically detect the best option, but manual configuration is also possible and more reliable.")
)]
#![doc = ""]
#![cfg_attr(
    psram_code_in_psram,
    doc = "With the `code-in-psram` option, the start-up starts PSRAM and runs the code from it. \
           `#[psram(zeroed)]` places a static in PSRAM, which the start-up zeroes."
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

#[cfg(any(esp32s2, esp32s3))]
mod quad_xtensa;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(any(esp32c5, esp32c61, esp32h4), path = "esp32c5_c61.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
#[cfg_attr(esp32s31, path = "esp32s31.rs")]
pub(crate) mod implem;

#[cfg(code_in_psram)]
pub(crate) mod code_in_psram;

pub use implem::*;
use portable_atomic::{AtomicUsize, Ordering};

use crate::peripherals::PSRAM;

/// Size of PSRAM.
///
/// [`PsramSize::AutoDetect`] tries to detect the size of PSRAM.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramSize {
    /// Detect PSRAM size
    #[default]
    AutoDetect,
    /// A fixed PSRAM size.
    Size(usize),
}

// Where flash and PSRAM share one MMU, the start-up starts PSRAM through the driver's own
// `PsramConfig`.
#[cfg(any(not(code_in_psram), not(psram_separate_window)))]
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

const EXTMEM_ORIGIN: usize = property!("psram.extmem_origin");

static MAPPED_PSRAM_START: AtomicUsize = AtomicUsize::new(0);
static MAPPED_PSRAM_END: AtomicUsize = AtomicUsize::new(0);

/// Returns the range that `Psram::raw_parts` gives: the PSRAM after the code and the PSRAM statics.
#[inline(always)]
pub(crate) fn psram_heap_range() -> Range<usize> {
    let end = MAPPED_PSRAM_END.load(Ordering::Acquire);
    let start = MAPPED_PSRAM_START.load(Ordering::Relaxed);
    if end < start { 0..0 } else { start..end }
}

/// Returns the addresses that hold data in PSRAM: the heap, and with code in PSRAM the PSRAM
/// statics before it. DMA treats a buffer in this range as PSRAM.
#[inline(always)]
pub(crate) fn psram_range() -> Range<usize> {
    let heap = psram_heap_range();
    cfg_select! {
        all(code_in_psram, psram_separate_window) => {
            let start = &raw const code_in_psram::_psram_bss_start as usize;
            let end = &raw const code_in_psram::_psram_bss_end as usize;
            if start == end {
                heap
            } else if heap.is_empty() {
                start..end
            } else {
                start..heap.end
            }
        }
        _ => heap,
    }
}

/// Returns whether PSRAM holds anything that must survive a sleep: the heap, or with code in
/// PSRAM the code and the PSRAM statics, even when the heap is empty.
#[allow(unused)]
#[inline(always)]
pub(crate) fn in_use() -> bool {
    cfg!(code_in_psram) || !psram_range().is_empty()
}

/// # Safety
///
/// Must only be called once.
unsafe fn set_psram_range(range: Range<usize>) {
    MAPPED_PSRAM_START.store(range.start, Ordering::Relaxed);
    MAPPED_PSRAM_END.store(range.end, Ordering::Release);
}

/// Enables externally-connected Pseudo-static RAM.
pub struct Psram {
    _peri: PSRAM<'static>,
}

/// Records the PSRAM after the code and the PSRAM statics, which `Psram::raw_parts` returns.
/// `esp_hal::init` calls it, after the start-up in [`code_in_psram`] wrote its record.
///
/// # Panics
///
/// Panics if `.bss.psram` ends past the end of PSRAM. The start-up then left it unzeroed.
#[cfg(code_in_psram)]
pub(crate) fn start_with_code() {
    implem::request_clock();

    let size = code_in_psram::SIZE.load(Ordering::Relaxed);
    info!("PSRAM size: {} MB", size / 1_024 / 1_024);
    if code_in_psram::flash_encrypted() {
        warn!("The code copied to PSRAM is plaintext: esp-hal does not encrypt PSRAM");
    }

    // The heap takes the window after the code and `.bss.psram`.
    #[cfg(psram_separate_window)]
    let range = {
        const PAGE: usize = property!("mmu.page_size");

        let end = EXTMEM_ORIGIN + size;
        let bss_end = &raw const code_in_psram::_psram_bss_end as usize;
        if bss_end > end {
            panic!(
                "PSRAM statics end at {:#x}, but PSRAM is {} MB",
                bss_end,
                size / 1_024 / 1_024
            );
        }
        let used = code_in_psram::CODE_END.load(Ordering::Relaxed).max(bss_end);
        used.next_multiple_of(PAGE)..end
    };
    // The heap needs entries of its own, after the code's.
    #[cfg(not(psram_separate_window))]
    let range = code_in_psram::map_heap(size);
    unsafe { set_psram_range(range) };
}

impl Psram {
    /// Initializes PSRAM.
    #[cfg_attr(
        psram_code_in_psram,
        doc = "\n\nWith `code-in-psram`, PSRAM is already started before `esp_hal::init`. `new` logs \
               a warning if `config` is not the default, and PSRAM does not change."
    )]
    pub fn new(peri: PSRAM<'static>, config: PsramConfig) -> Self {
        cfg_select! {
            code_in_psram => {
                if config != PsramConfig::default() {
                    warn!(
                        "PSRAM runs from start-up with `psram-frequency`, and ignores this config"
                    );
                }
            }
            _ => {
                let mut config = config;
                if init_psram(&mut config) {
                    info!("PSRAM size: {} MB", config.size.get() / 1_024 / 1_024);
                    let range = map_psram(config);

                    unsafe { set_psram_range(range) };

                    #[cfg(all(cache_livelock_workaround, feature = "rt"))]
                    crate::soc::cache_livelock::psram_mapped();
                }
            }
        }
        Self { _peri: peri }
    }

    /// Returns the address and size of the available in external memory.
    pub fn raw_parts(&self) -> (*mut u8, usize) {
        let range = psram_heap_range();
        (range.start as *mut u8, range.end - range.start)
    }

    /// Returns the PSRAM size in bytes.
    #[cfg_attr(
        psram_code_in_psram,
        doc = "\n\nWith `code-in-psram`, the range from [`Self::raw_parts`] is smaller, because the \
               code and the PSRAM statics come first."
    )]
    #[instability::unstable]
    pub fn size(&self) -> usize {
        cfg_select! {
            code_in_psram => code_in_psram::SIZE.load(Ordering::Relaxed),
            _ => psram_range().len(),
        }
    }
}

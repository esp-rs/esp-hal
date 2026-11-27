//! Save and restore of the CPU-domain device registers.
//!
//! esp-idf moves these with plain register loops and not with the PAU regdma engine
//! (`esp32c6/sleep_cpu.c:237-258`).

use procmacros::ram;

use crate::private::panic_free::{load_word, store_word};

/// A run of device registers that the CPU power-down loses.
pub(crate) struct DeviceRegion {
    start: *const u32,
    words: usize,
}

impl DeviceRegion {
    pub(crate) const fn new(start: *const u32, words: usize) -> Self {
        Self { start, words }
    }
}

/// Reads every region into `frame`, in the order of `regions`.
///
/// Stops at the end of `frame` if the regions hold more words.
#[ram(unstable(checked))]
pub(crate) fn save(regions: &[DeviceRegion], frame: &mut [u32]) {
    // Do not use slice iterators. With debug assertions, they check the pointer arithmetic, and
    // the panic path is in flash.
    let mut slot = 0;
    let mut r = 0;
    while r < regions.len() {
        let region = &regions[r];
        let mut word = 0;
        while word < region.words {
            if slot >= frame.len() {
                return;
            }
            // SAFETY: a chip module builds every region from a register block that the chip has.
            frame[slot] = unsafe { load_word(region.start.wrapping_add(word)) };
            slot += 1;
            word += 1;
        }
        r += 1;
    }
}

/// Writes `frame` back to every region, in the order of `regions`.
///
/// Stops at the end of `frame` if the regions hold more words.
#[ram(unstable(checked))]
pub(crate) fn restore(regions: &[DeviceRegion], frame: &[u32]) {
    // Do not use slice iterators. See `save`.
    let mut slot = 0;
    let mut r = 0;
    while r < regions.len() {
        let region = &regions[r];
        let mut word = 0;
        while word < region.words {
            if slot >= frame.len() {
                return;
            }
            // SAFETY: a chip module builds every region from a register block that the chip has.
            unsafe { store_word(region.start.wrapping_add(word).cast_mut(), frame[slot]) };
            slot += 1;
            word += 1;
        }
        r += 1;
    }
}

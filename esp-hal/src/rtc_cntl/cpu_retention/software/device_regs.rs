//! Save and restore of the CPU-domain device registers.
//!
//! esp-idf moves these with plain register loops and not with the PAU regdma engine
//! (`esp32c6/sleep_cpu.c:237-258`).

use procmacros::ram;

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
#[ram(unstable(caches_suspended))]
pub(crate) fn save(regions: &[DeviceRegion], frame: &mut [u32]) {
    let mut slots = frame.iter_mut();
    for region in regions {
        for word in 0..region.words {
            let Some(slot) = slots.next() else {
                return;
            };
            // SAFETY: a chip module builds every region from a register block that the chip has.
            *slot = unsafe { load_word(region.start.wrapping_add(word)) };
        }
    }
}

/// Writes `frame` back to every region, in the order of `regions`.
///
/// Stops at the end of `frame` if the regions hold more words.
#[ram(unstable(caches_suspended))]
pub(crate) fn restore(regions: &[DeviceRegion], frame: &[u32]) {
    let mut slots = frame.iter();
    for region in regions {
        for word in 0..region.words {
            let Some(slot) = slots.next() else {
                return;
            };
            // SAFETY: a chip module builds every region from a register block that the chip has.
            unsafe { store_word(region.start.wrapping_add(word).cast_mut(), *slot) };
        }
    }
}

/// Reads one word with `lw`.
///
/// With debug assertions, `read_volatile` checks the alignment, and the panic path is in flash.
/// This function runs with the caches suspended.
///
/// # Safety
///
/// `addr` must be a readable, aligned word.
#[inline(always)]
pub(crate) unsafe fn load_word(addr: *const u32) -> u32 {
    let value;
    unsafe {
        core::arch::asm!(
            "lw {0}, 0({1})",
            out(reg) value,
            in(reg) addr,
            options(nostack, readonly, preserves_flags),
        );
    }
    value
}

/// Writes one word with `sw`. See [`load_word`] for the reason.
///
/// # Safety
///
/// `addr` must be a writable, aligned word.
#[inline(always)]
pub(crate) unsafe fn store_word(addr: *mut u32, value: u32) {
    unsafe {
        core::arch::asm!(
            "sw {0}, 0({1})",
            in(reg) value,
            in(reg) addr,
            options(nostack, preserves_flags),
        );
    }
}

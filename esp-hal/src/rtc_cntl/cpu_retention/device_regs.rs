//! Save and restore of the CPU-domain device registers.
//!
//! esp-idf moves these with plain register loops and not with the PAU regdma engine
//! (`esp32c6/sleep_cpu.c:237-258`).

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
/// # Panics
///
/// Panics if `frame` is shorter than the regions need.
#[crate::ram]
pub(crate) fn save(regions: &[DeviceRegion], frame: &mut [u32]) {
    let mut offset = 0;
    for region in regions {
        for word in 0..region.words {
            // SAFETY: a chip module builds every region from a register block that the chip has.
            frame[offset] = unsafe { region.start.add(word).read_volatile() };
            offset += 1;
        }
    }
}

/// Writes `frame` back to every region, in the order of `regions`.
///
/// # Panics
///
/// Panics if `frame` is shorter than the regions need.
#[crate::ram]
pub(crate) fn restore(regions: &[DeviceRegion], frame: &[u32]) {
    let mut offset = 0;
    for region in regions {
        for word in 0..region.words {
            // SAFETY: a chip module builds every region from a register block that the chip has.
            unsafe {
                region
                    .start
                    .add(word)
                    .cast_mut()
                    .write_volatile(frame[offset])
            };
            offset += 1;
        }
    }
}

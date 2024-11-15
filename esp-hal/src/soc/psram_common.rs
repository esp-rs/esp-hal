use core::ops::Range;

/// Size of PSRAM
///
/// [PsramSize::AutoDetect] will try to detect the size of PSRAM
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// Returns the address and size of the available in external memory.
#[cfg(any(feature = "quad-psram", feature = "octal-psram"))]
pub fn psram_raw_parts(psram: &crate::peripherals::PSRAM) -> (*mut u8, usize) {
    let range = psram_range(psram);
    (range.start as *mut u8, range.end - range.start)
}

#![no_std]

/// Bytes in `.dram2_uninit` for the selected chip.
pub const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

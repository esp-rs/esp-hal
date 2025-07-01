//! Definitions of ROM functions related to flash memory.

/// The operation succeeded
pub const ESP_ROM_SPIFLASH_RESULT_OK: i32 = 0;

/// The operation errored
pub const ESP_ROM_SPIFLASH_RESULT_ERR: i32 = 1;

/// The operation timed out
pub const ESP_ROM_SPIFLASH_RESULT_TIMEOUT: i32 = 2;

unsafe extern "C" {
    /// Read Data from Flash via ROM code, you should Erase it yourself if need.
    ///
    /// `src_addr` should be 4 bytes aligned.
    /// `len` should be 4 bytes aligned.
    pub fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32;

    /// Clear all SR bits except QE bit.
    pub fn esp_rom_spiflash_unlock() -> i32;

    /// Erase a sector of flash. Uses SPI flash command 20H.
    pub fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32;

    /// Write Data to Flash, you should Erase it yourself if need.
    ///
    /// `dest_addr` should be 4 bytes aligned.
    /// `len` should be 4 bytes aligned.
    pub fn esp_rom_spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32;
}

//! # Low-level API
//!
//! ⚠️ This is a low-level API and should be used with caution. ⚠️
//!
//! This gives you access to the underlying low level functionality.
//! These operate on raw pointers and all functions here are unsafe.
//! No pre-conditions are checked by any of these functions.

use crate::chip_specific;

/// Low-level SPI NOR Flash read
///
/// # Safety
///
/// The `src_addr` + `len` should not exceeds the size of flash.
/// The `data` expected to points to word-aligned pre-allocated buffer with size
/// greater or equals to `len`.
pub unsafe fn spiflash_read(src_addr: u32, data: *mut u32, len: u32) -> Result<(), i32> {
    match chip_specific::spiflash_read(src_addr, data, len) {
        0 => Ok(()),
        value => Err(value),
    }
}

/// Low-level SPI NOR Flash unlock
///
/// # Safety
pub unsafe fn spiflash_unlock() -> Result<(), i32> {
    match chip_specific::spiflash_unlock() {
        0 => Ok(()),
        value => Err(value),
    }
}

/// Low-level SPI NOR Flash erase
///
/// # Safety
///
/// The `sector_number` * sector_size should not exceeds the size of flash.
pub unsafe fn spiflash_erase_sector(sector_number: u32) -> Result<(), i32> {
    match chip_specific::spiflash_erase_sector(sector_number) {
        0 => Ok(()),
        value => Err(value),
    }
}

/// Low-level SPI NOR Flash write
///
/// # Safety
///
/// The `dest_addr` + `len` should not exceeds the size of flash.
/// The `data` expected to points to word-aligned buffer with size greater or
/// equals to `len`.
pub unsafe fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> Result<(), i32> {
    match chip_specific::spiflash_write(dest_addr, data, len) {
        0 => Ok(()),
        value => Err(value),
    }
}

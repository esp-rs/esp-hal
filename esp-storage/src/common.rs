use core::ops::{Deref, DerefMut};

use crate::chip_specific;

#[repr(C, align(4))]
pub struct FlashSectorBuffer {
    // NOTE: Ensure that no unaligned fields are added above `data` to maintain its required
    // alignment
    data: [u8; FlashStorage::SECTOR_SIZE as usize],
}

impl Deref for FlashSectorBuffer {
    type Target = [u8; FlashStorage::SECTOR_SIZE as usize];

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl DerefMut for FlashSectorBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum FlashStorageError {
    IoError,
    IoTimeout,
    CantUnlock,
    NotAligned,
    OutOfBounds,
    Other(i32),
}

#[inline(always)]
pub fn check_rc(rc: i32) -> Result<(), FlashStorageError> {
    match rc {
        0 => Ok(()),
        1 => Err(FlashStorageError::IoError),
        2 => Err(FlashStorageError::IoTimeout),
        _ => Err(FlashStorageError::Other(rc)),
    }
}

#[derive(Debug)]
pub struct FlashStorage {
    pub(crate) capacity: usize,
    unlocked: bool,
}

impl Default for FlashStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashStorage {
    pub const WORD_SIZE: u32 = 4;
    pub const SECTOR_SIZE: u32 = 4096;

    pub fn new() -> FlashStorage {
        let mut storage = FlashStorage {
            capacity: 0,
            unlocked: false,
        };

        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        const ADDR: u32 = 0x0000;
        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        const ADDR: u32 = 0x1000;

        let mut buffer = [0u8; 8];
        storage.internal_read(ADDR, &mut buffer).ok();
        let mb = match buffer[3] & 0xf0 {
            0x00 => 1,
            0x10 => 2,
            0x20 => 4,
            0x30 => 8,
            0x40 => 16,
            0x50 => 32,
            _ => 0,
        };
        storage.capacity = mb * 1024 * 1024;

        storage
    }

    #[cfg(feature = "nor-flash")]
    #[inline(always)]
    pub(crate) fn check_alignment<const ALIGN: u32>(
        &self,
        offset: u32,
        length: usize,
    ) -> Result<(), FlashStorageError> {
        let offset = offset as usize;
        if offset % ALIGN as usize != 0 || length % ALIGN as usize != 0 {
            return Err(FlashStorageError::NotAligned);
        }
        Ok(())
    }

    #[inline(always)]
    pub(crate) fn check_bounds(&self, offset: u32, length: usize) -> Result<(), FlashStorageError> {
        let offset = offset as usize;
        if length > self.capacity || offset > self.capacity - length {
            return Err(FlashStorageError::OutOfBounds);
        }
        Ok(())
    }

    #[allow(clippy::all)]
    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub(crate) fn internal_read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), FlashStorageError> {
        check_rc(chip_specific::spiflash_read(
            offset,
            bytes.as_ptr() as *mut u32,
            bytes.len() as u32,
        ))
    }

    #[inline(always)]
    fn unlock_once(&mut self) -> Result<(), FlashStorageError> {
        if !self.unlocked {
            if chip_specific::spiflash_unlock() != 0 {
                return Err(FlashStorageError::CantUnlock);
            }
            self.unlocked = true;
        }
        Ok(())
    }

    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub(crate) fn internal_erase(&mut self, sector: u32) -> Result<(), FlashStorageError> {
        self.unlock_once()?;

        check_rc(chip_specific::spiflash_erase_sector(sector))
    }

    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub(crate) fn internal_write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        self.unlock_once()?;

        check_rc(chip_specific::spiflash_write(
            offset,
            bytes.as_ptr() as *const u32,
            bytes.len() as u32,
        ))
    }
}

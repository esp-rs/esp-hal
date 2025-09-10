use core::mem::MaybeUninit;

use crate::chip_specific;
#[cfg(multi_core)]
use crate::multi_core::MultiCoreStrategy;

#[derive(Debug)]
#[non_exhaustive]
pub enum FlashStorageError {
    IoError,
    IoTimeout,
    CantUnlock,
    NotAligned,
    OutOfBounds,
    /// Cannot write to flash as more than one core is running.
    /// Either manually suspend the other core, or use one of the available strategies:
    /// * [`FlashStorage::multicore_auto_park`]
    /// * [`FlashStorage::multicore_ignore`]
    #[cfg(multi_core)]
    OtherCoreRunning,
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
    #[cfg(multi_core)]
    pub(crate) multi_core_strategy: MultiCoreStrategy,
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
            #[cfg(multi_core)]
            multi_core_strategy: MultiCoreStrategy::Error,
        };

        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        const ADDR: u32 = 0x0000;
        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        const ADDR: u32 = 0x1000;

        let mut buffer = crate::buffer::FlashWordBuffer::uninit();
        storage.internal_read(ADDR, buffer.as_bytes_mut()).unwrap();
        let buffer = unsafe { buffer.assume_init_bytes() };
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

    pub(crate) fn internal_read(
        &mut self,
        offset: u32,
        bytes: &mut [MaybeUninit<u8>],
    ) -> Result<(), FlashStorageError> {
        check_rc(chip_specific::spiflash_read(
            offset,
            bytes.as_mut_ptr() as *mut u32,
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

    pub(crate) fn internal_erase(&mut self, sector: u32) -> Result<(), FlashStorageError> {
        #[cfg(multi_core)]
        let unpark = self.multi_core_strategy.pre_write()?;

        self.unlock_once()?;
        check_rc(chip_specific::spiflash_erase_sector(sector))?;

        #[cfg(multi_core)]
        self.multi_core_strategy.post_write(unpark);

        Ok(())
    }

    pub(crate) fn internal_write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        #[cfg(multi_core)]
        let unpark = self.multi_core_strategy.pre_write()?;

        self.unlock_once()?;
        check_rc(chip_specific::spiflash_write(
            offset,
            bytes.as_ptr() as *const u32,
            bytes.len() as u32,
        ))?;

        #[cfg(multi_core)]
        self.multi_core_strategy.post_write(unpark);

        Ok(())
    }
}

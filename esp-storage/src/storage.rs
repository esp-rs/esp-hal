use core::mem::MaybeUninit;

use embedded_storage::{ReadStorage, Storage};

use crate::{FlashSectorBuffer, FlashStorage, FlashStorageError};

impl ReadStorage for FlashStorage {
    type Error = FlashStorageError;

    fn read(&mut self, offset: u32, mut bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::WORD_SIZE;
        let mut aligned_offset = offset - data_offset;

        // Bypass clearing sector buffer for performance reasons
        let mut sector_data = MaybeUninit::<FlashSectorBuffer>::uninit();
        let sector_data = unsafe { sector_data.assume_init_mut() };

        while !bytes.is_empty() {
            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as _);

            let aligned_end = (data_offset as usize + len + (Self::WORD_SIZE - 1) as usize)
                & !(Self::WORD_SIZE - 1) as usize;

            // Read only needed data words
            self.internal_read(aligned_offset, &mut sector_data[..aligned_end])?;

            bytes[..len].copy_from_slice(&sector_data[data_offset as usize..][..len]);

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &mut bytes[len..];
        }

        Ok(())
    }

    /// The SPI flash size is configured by writing a field in the software
    /// bootloader image header. This is done during flashing in espflash /
    /// esptool.
    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl Storage for FlashStorage {
    fn write(&mut self, offset: u32, mut bytes: &[u8]) -> Result<(), Self::Error> {
        self.check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::SECTOR_SIZE;
        let mut aligned_offset = offset - data_offset;

        // Bypass clearing sector buffer for performance reasons
        let mut sector_data = MaybeUninit::<FlashSectorBuffer>::uninit();
        let sector_data = unsafe { sector_data.assume_init_mut() };

        while !bytes.is_empty() {
            self.internal_read(aligned_offset, &mut sector_data[..])?;

            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as _);

            sector_data[data_offset as usize..][..len].copy_from_slice(&bytes[..len]);
            self.internal_erase(aligned_offset / Self::SECTOR_SIZE)?;
            self.internal_write(aligned_offset, &sector_data[..])?;

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &bytes[len..];
        }

        Ok(())
    }
}

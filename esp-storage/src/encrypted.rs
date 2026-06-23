use crate::{FlashStorage, FlashStorageError, buffer::FlashSectorBuffer};

impl FlashStorage<'_> {
    /// Read bytes from encrypted flash.
    ///
    /// Uses the MMU to map flash pages and reads decrypted data through the cache.
    /// Unaligned offsets and lengths are supported.
    ///
    /// If flash encryption is not enabled this will just read plaintext.
    pub fn read_encrypted(
        &mut self,
        offset: u32,
        mut bytes: &mut [u8],
    ) -> Result<(), FlashStorageError> {
        self.check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::SECTOR_SIZE;
        let mut aligned_offset = offset - data_offset;

        let mut sector_data = FlashSectorBuffer::uninit();

        while !bytes.is_empty() {
            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as _);

            self.internal_read_encrypted(aligned_offset, sector_data.as_bytes_mut())?;
            let sector_data = unsafe { sector_data.assume_init_bytes_mut() };
            bytes[..len].copy_from_slice(&sector_data[data_offset as usize..][..len]);

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &mut bytes[len..];
        }

        Ok(())
    }

    /// Write bytes to encrypted flash.
    ///
    /// Performs read-modify-write on affected sectors: reads the current encrypted
    /// content, merges the new bytes, erases the sector, then writes it back encrypted.
    ///
    /// # Errors
    /// - [FlashStorageError::NotSupported] if flash encryption is not enabled
    pub fn write_encrypted(
        &mut self,
        offset: u32,
        mut bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        // we have a HIL test which exploits the fact that the ROM function
        // will actually do encryption even if the flash encryption isn't enabled via efuse
        #[cfg(not(any(__test_esp_storage, feature = "emulation")))]
        if !crate::flash_encryption() {
            return Err(FlashStorageError::NotSupported);
        }

        self.check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::SECTOR_SIZE;
        let mut aligned_offset = offset - data_offset;

        let mut sector_data = FlashSectorBuffer::uninit();

        while !bytes.is_empty() {
            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as _);

            self.internal_read_encrypted(aligned_offset, sector_data.as_bytes_mut())?;
            let sector_data = unsafe { sector_data.assume_init_bytes_mut() };

            sector_data[data_offset as usize..][..len].copy_from_slice(&bytes[..len]);
            self.internal_erase_sector(aligned_offset / Self::SECTOR_SIZE)?;
            self.internal_write_encrypted(aligned_offset, sector_data)?;

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &bytes[len..];
        }

        Ok(())
    }
}

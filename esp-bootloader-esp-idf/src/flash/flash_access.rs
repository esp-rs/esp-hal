use crate::partitions::Error;

/// Internal flash I/O trait used by partition and OTA logic.
#[doc(hidden)]
pub trait FlashAccess {
    fn flash_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error>;
    fn flash_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error>;
    fn flash_erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
    fn flash_read_encrypted(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error>;
    fn flash_write_encrypted(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error>;

    #[cfg(feature = "embedded-storage")]
    const READ_SIZE: usize;
    #[cfg(feature = "embedded-storage")]
    const WRITE_SIZE: usize;
    #[cfg(feature = "embedded-storage")]
    const ERASE_SIZE: usize;
    #[cfg(feature = "embedded-storage")]
    const SECTOR_SIZE: u32;

    #[cfg(feature = "embedded-storage")]
    fn flash_read_nor(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error>;
    #[cfg(feature = "embedded-storage")]
    fn flash_write_nor(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error>;
}

#[cfg(not(feature = "std"))]
impl FlashAccess for super::FlashStorage<'_> {
    #[cfg(feature = "embedded-storage")]
    const READ_SIZE: usize = esp_storage::FlashStorage::READ_SIZE;
    #[cfg(feature = "embedded-storage")]
    const WRITE_SIZE: usize = esp_storage::FlashStorage::WRITE_SIZE;
    #[cfg(feature = "embedded-storage")]
    const ERASE_SIZE: usize = esp_storage::FlashStorage::ERASE_SIZE;
    #[cfg(feature = "embedded-storage")]
    const SECTOR_SIZE: u32 = esp_storage::FlashStorage::SECTOR_SIZE;

    fn flash_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        esp_storage::FlashStorage::read(self, offset, bytes).map_err(|_| Error::StorageError)
    }

    fn flash_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        esp_storage::FlashStorage::write(self, offset, bytes).map_err(|_| Error::StorageError)
    }

    fn flash_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        esp_storage::FlashStorage::erase(self, from, to).map_err(|_| Error::StorageError)
    }

    fn flash_read_encrypted(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        esp_storage::FlashStorage::read_encrypted(self, offset, bytes)
            .map_err(|_| Error::StorageError)
    }

    fn flash_write_encrypted(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        esp_storage::FlashStorage::write_encrypted(self, offset, bytes)
            .map_err(|_| Error::StorageError)
    }

    #[cfg(feature = "embedded-storage")]
    fn flash_read_nor(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        esp_storage::FlashStorage::read_nor(self, offset, bytes).map_err(|_| Error::StorageError)
    }

    #[cfg(feature = "embedded-storage")]
    fn flash_write_nor(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        esp_storage::FlashStorage::write_nor(self, offset, bytes).map_err(|_| Error::StorageError)
    }
}

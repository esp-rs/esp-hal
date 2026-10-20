use crate::partitions::Error;

/// Internal flash I/O trait used by partition and OTA logic.
#[doc(hidden)]
pub trait FlashAccess {
    fn flash_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error>;
    fn flash_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error>;
    fn flash_erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
    fn flash_read_encrypted(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error>;
    fn flash_write_encrypted(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error>;
}

#[cfg(not(feature = "std"))]
mod esp_hal_flash {
    use super::*;
    use crate::flash::{FlashStorage, WORD_SIZE};

    /// Size of the stack buffer that stages data between byte slices and the
    /// word-based flash driver, which needs word-aligned buffers in DRAM.
    const BOUNCE_BUFFER_WORDS: usize = 64;
    const BOUNCE_BUFFER_BYTES: usize = BOUNCE_BUFFER_WORDS * WORD_SIZE as usize;

    fn words_as_bytes(words: &[u32]) -> &[u8] {
        // SAFETY: every bit pattern is a valid `u8`, and `u8` has no alignment requirement.
        unsafe { core::slice::from_raw_parts(words.as_ptr().cast(), size_of_val(words)) }
    }

    fn words_as_bytes_mut(words: &mut [u32]) -> &mut [u8] {
        // SAFETY: every bit pattern is a valid `u8` and `u32`, and `u8` has no alignment
        // requirement.
        unsafe { core::slice::from_raw_parts_mut(words.as_mut_ptr().cast(), size_of_val(words)) }
    }

    /// Reads `bytes.len()` bytes at any `offset`, one bounce buffer at a time.
    fn read_bytes(
        flash: &mut FlashStorage<'_>,
        offset: u32,
        mut bytes: &mut [u8],
        encrypted: bool,
    ) -> Result<(), Error> {
        let mut buffer = [0u32; BOUNCE_BUFFER_WORDS];
        let mut address = offset;

        while !bytes.is_empty() {
            let lead = (address % WORD_SIZE) as usize;
            let len = bytes.len().min(BOUNCE_BUFFER_BYTES - lead);
            let words = &mut buffer[..(lead + len).div_ceil(WORD_SIZE as usize)];
            let aligned_address = address - lead as u32;

            if encrypted {
                flash.read_encrypted(aligned_address, words)
            } else {
                flash.read(aligned_address, words)
            }
            .map_err(|_| Error::StorageError)?;

            let (head, tail) = bytes.split_at_mut(len);
            head.copy_from_slice(&words_as_bytes(words)[lead..][..len]);
            bytes = tail;
            address += len as u32;
        }

        Ok(())
    }

    /// Writes `bytes` at a word-aligned `offset`, one bounce buffer at a time.
    ///
    /// The target range must be erased: the driver does not erase before
    /// programming.
    fn write_bytes(
        flash: &mut FlashStorage<'_>,
        offset: u32,
        mut bytes: &[u8],
        encrypted: bool,
    ) -> Result<(), Error> {
        if !offset.is_multiple_of(WORD_SIZE) || !bytes.len().is_multiple_of(WORD_SIZE as usize) {
            return Err(Error::NotAligned);
        }

        let mut buffer = [0u32; BOUNCE_BUFFER_WORDS];
        let mut address = offset;

        while !bytes.is_empty() {
            let len = bytes.len().min(BOUNCE_BUFFER_BYTES);
            let words = &mut buffer[..len / WORD_SIZE as usize];
            let (head, tail) = bytes.split_at(len);
            words_as_bytes_mut(words).copy_from_slice(head);

            // SAFETY: all writes and erases go through `Region`, which refuses to
            // modify the partition of the running application, the only flash that
            // is mapped for instruction fetch or as immutable data after boot.
            unsafe {
                if encrypted {
                    flash.write_encrypted(address, words)
                } else {
                    flash.write(address, words)
                }
            }
            .map_err(|_| Error::StorageError)?;

            bytes = tail;
            address += len as u32;
        }

        Ok(())
    }

    impl FlashAccess for FlashStorage<'_> {
        fn flash_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
            read_bytes(self, offset, bytes, false)
        }

        fn flash_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
            write_bytes(self, offset, bytes, false)
        }

        fn flash_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
            // SAFETY: all writes and erases go through `Region`, which refuses to
            // modify the partition of the running application, the only flash that
            // is mapped for instruction fetch or as immutable data after boot.
            unsafe { self.erase(from, to) }.map_err(|_| Error::StorageError)
        }

        fn flash_read_encrypted(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
            read_bytes(self, offset, bytes, true)
        }

        fn flash_write_encrypted(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
            write_bytes(self, offset, bytes, true)
        }
    }
}

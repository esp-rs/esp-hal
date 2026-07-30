use core::marker::PhantomData;

use super::flash_access::FlashAccess;
use crate::partitions::Error;

const WORD_SIZE: u32 = 4;
const SECTOR_SIZE: u32 = 4096;
const BLOCK_SIZE: u32 = 65536;
const FLASH_SIZE: usize = (BLOCK_SIZE * 4) as usize;
const ERASE_BYTE: u8 = 0xff;

static mut FLASH_DATA: [u8; FLASH_SIZE] = [ERASE_BYTE; FLASH_SIZE];

/// In-memory flash used by host tests (`std` feature).
#[derive(Debug)]
pub struct MockFlash<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl MockFlash<'static> {
    pub fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

impl MockFlash<'_> {
    fn with_flash<R>(f: impl FnOnce(&mut [u8; FLASH_SIZE]) -> R) -> R {
        // Host tests run with `--test-threads=1`.
        f(unsafe { &mut *core::ptr::addr_of_mut!(FLASH_DATA) })
    }

    fn check_bounds(offset: u32, length: usize) -> Result<(), Error> {
        let offset = offset as usize;
        if length > FLASH_SIZE || offset > FLASH_SIZE - length {
            return Err(Error::StorageError);
        }
        Ok(())
    }

    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.flash_read(offset, bytes)
    }

    pub fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.flash_write(offset, bytes)
    }

    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.flash_erase(from, to)
    }
}

impl FlashAccess for MockFlash<'_> {
    const READ_SIZE: usize = WORD_SIZE as usize;
    const WRITE_SIZE: usize = WORD_SIZE as usize;
    const ERASE_SIZE: usize = SECTOR_SIZE as usize;
    const SECTOR_SIZE: u32 = SECTOR_SIZE;

    fn flash_read(&mut self, offset: u32, mut bytes: &mut [u8]) -> Result<(), Error> {
        Self::check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::SECTOR_SIZE;
        let mut aligned_offset = offset - data_offset;

        while !bytes.is_empty() {
            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as usize);

            Self::with_flash(|flash| {
                bytes[..len].copy_from_slice(
                    &flash[aligned_offset as usize..][data_offset as usize..][..len],
                );
            });

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &mut bytes[len..];
        }

        Ok(())
    }

    fn flash_write(&mut self, offset: u32, mut bytes: &[u8]) -> Result<(), Error> {
        Self::check_bounds(offset, bytes.len())?;

        let mut data_offset = offset % Self::SECTOR_SIZE;
        let mut aligned_offset = offset - data_offset;

        while !bytes.is_empty() {
            let len = bytes.len().min((Self::SECTOR_SIZE - data_offset) as usize);

            Self::with_flash(|flash| {
                let sector = aligned_offset as usize;
                let sector_end = sector + Self::SECTOR_SIZE as usize;
                flash[sector..sector_end].fill(ERASE_BYTE);
                flash[sector + data_offset as usize..][..len].copy_from_slice(&bytes[..len]);
            });

            aligned_offset += Self::SECTOR_SIZE;
            data_offset = 0;
            bytes = &bytes[len..];
        }

        Ok(())
    }

    fn flash_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let len = (to - from) as usize;
        Self::check_bounds(from, len)?;

        let mut address = from;
        while address < to && !address.is_multiple_of(BLOCK_SIZE) {
            let sector = address as usize;
            Self::with_flash(|flash| {
                flash[sector..sector + Self::SECTOR_SIZE as usize].fill(ERASE_BYTE);
            });
            address += Self::SECTOR_SIZE;
        }

        while (to - address) >= BLOCK_SIZE {
            let block = address as usize;
            Self::with_flash(|flash| {
                flash[block..block + BLOCK_SIZE as usize].fill(ERASE_BYTE);
            });
            address += BLOCK_SIZE;
        }

        while address < to {
            let sector = address as usize;
            Self::with_flash(|flash| {
                flash[sector..sector + Self::SECTOR_SIZE as usize].fill(ERASE_BYTE);
            });
            address += Self::SECTOR_SIZE;
        }

        Ok(())
    }

    fn flash_read_encrypted(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.flash_read(offset, bytes)
    }

    fn flash_write_encrypted(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.flash_write(offset, bytes)
    }

    fn flash_read_nor(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.flash_read(offset, bytes)
    }

    fn flash_write_nor(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.flash_write(offset, bytes)
    }
}

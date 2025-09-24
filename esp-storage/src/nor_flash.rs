use embedded_storage::nor_flash::{
    ErrorType,
    MultiwriteNorFlash,
    NorFlash,
    NorFlashError,
    NorFlashErrorKind,
    ReadNorFlash,
};

#[cfg(feature = "bytewise-read")]
use crate::buffer::FlashWordBuffer;
use crate::{
    FlashStorage,
    FlashStorageError,
    buffer::{FlashSectorBuffer, uninit_slice, uninit_slice_mut},
};

impl FlashStorage<'_> {
    #[inline(always)]
    fn is_word_aligned(bytes: &[u8]) -> bool {
        // TODO: Use is_aligned_to when stabilized (see `pointer_is_aligned`)
        (bytes.as_ptr() as usize) % (Self::WORD_SIZE as usize) == 0
    }
}

impl NorFlashError for FlashStorageError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

impl ErrorType for FlashStorage<'_> {
    type Error = FlashStorageError;
}

impl ReadNorFlash for FlashStorage<'_> {
    #[cfg(not(feature = "bytewise-read"))]
    const READ_SIZE: usize = Self::WORD_SIZE as _;

    #[cfg(feature = "bytewise-read")]
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        const RS: u32 = FlashStorage::READ_SIZE as u32;
        self.check_alignment::<{ RS }>(offset, bytes.len())?;
        self.check_bounds(offset, bytes.len())?;

        #[cfg(feature = "bytewise-read")]
        let (offset, bytes) = {
            let byte_offset = (offset % Self::WORD_SIZE) as usize;
            if byte_offset > 0 {
                let mut word_buffer = FlashWordBuffer::uninit();

                let offset = offset - byte_offset as u32;
                let length = bytes.len().min(Self::WORD_SIZE as usize - byte_offset);

                self.internal_read(offset, word_buffer.as_bytes_mut())?;
                let word_buffer = unsafe { word_buffer.assume_init_bytes_mut() };
                bytes[..length].copy_from_slice(&word_buffer[byte_offset..][..length]);

                (offset + Self::WORD_SIZE, &mut bytes[length..])
            } else {
                (offset, bytes)
            }
        };

        if Self::is_word_aligned(bytes) {
            // Bytes buffer is word-aligned so we can read directly to it
            for (offset, chunk) in (offset..)
                .step_by(Self::SECTOR_SIZE as _)
                .zip(bytes.chunks_mut(Self::SECTOR_SIZE as _))
            {
                // Chunk already is word aligned so we can read directly to it
                #[cfg(not(feature = "bytewise-read"))]
                self.internal_read(offset, uninit_slice_mut(chunk))?;

                #[cfg(feature = "bytewise-read")]
                {
                    let length = chunk.len();
                    let byte_length = length % Self::WORD_SIZE as usize;
                    let length = length - byte_length;

                    self.internal_read(offset, &mut uninit_slice_mut(chunk)[..length])?;

                    // Read not aligned rest of data
                    if byte_length > 0 {
                        let mut word_buffer = FlashWordBuffer::uninit();

                        self.internal_read(offset + length as u32, word_buffer.as_bytes_mut())?;
                        let word_buffer = unsafe { word_buffer.assume_init_bytes_mut() };
                        chunk[length..].copy_from_slice(&word_buffer[..byte_length]);
                    }
                }
            }
        } else {
            // Bytes buffer isn't word-aligned so we might read only via aligned buffer
            let mut buffer = FlashSectorBuffer::uninit();

            for (offset, chunk) in (offset..)
                .step_by(Self::SECTOR_SIZE as _)
                .zip(bytes.chunks_mut(Self::SECTOR_SIZE as _))
            {
                // Read to temporary buffer first (chunk length is aligned)
                #[cfg(not(feature = "bytewise-read"))]
                self.internal_read(offset, &mut buffer.as_bytes_mut()[..chunk.len()])?;

                // Read to temporary buffer first (chunk length is not aligned)
                #[cfg(feature = "bytewise-read")]
                {
                    let length = chunk.len();
                    let byte_length = length % Self::WORD_SIZE as usize;
                    let length = if byte_length > 0 {
                        length - byte_length + Self::WORD_SIZE as usize
                    } else {
                        length
                    };

                    self.internal_read(offset, &mut buffer.as_bytes_mut()[..length])?;
                }
                let buffer = unsafe { buffer.assume_init_bytes() };

                // Copy to bytes buffer
                chunk.copy_from_slice(&buffer[..chunk.len()]);
            }
        }

        Ok(())
    }

    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl NorFlash for FlashStorage<'_> {
    const WRITE_SIZE: usize = Self::WORD_SIZE as _;
    const ERASE_SIZE: usize = Self::SECTOR_SIZE as _;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        const WS: u32 = FlashStorage::WORD_SIZE;
        self.check_alignment::<{ WS }>(offset, bytes.len())?;
        self.check_bounds(offset, bytes.len())?;

        if Self::is_word_aligned(bytes) {
            // Bytes buffer is word-aligned so we can write directly from it
            for (offset, chunk) in (offset..)
                .step_by(Self::SECTOR_SIZE as _)
                .zip(bytes.chunks(Self::SECTOR_SIZE as _))
            {
                // Chunk already is word aligned so we can write directly from it
                self.internal_write(offset, chunk)?;
            }
        } else {
            // Bytes buffer isn't word-aligned so we might write only via aligned buffer
            let mut buffer = FlashSectorBuffer::uninit();

            for (offset, chunk) in (offset..)
                .step_by(Self::SECTOR_SIZE as _)
                .zip(bytes.chunks(Self::SECTOR_SIZE as _))
            {
                // Copy to temporary buffer first
                buffer.as_bytes_mut()[..chunk.len()].copy_from_slice(uninit_slice(chunk));
                // Write from temporary buffer
                self.internal_write(offset, unsafe {
                    &buffer.assume_init_bytes()[..chunk.len()]
                })?;
            }
        }

        Ok(())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let len = (to - from) as _;
        const SZ: u32 = FlashStorage::SECTOR_SIZE;
        self.check_alignment::<{ SZ }>(from, len)?;
        self.check_bounds(from, len)?;

        for sector in from / Self::SECTOR_SIZE..to / Self::SECTOR_SIZE {
            self.internal_erase(sector)?;
        }

        Ok(())
    }
}

impl MultiwriteNorFlash for FlashStorage<'_> {}

// Run the tests with `--test-threads=1` - the emulation is not multithread safe
#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::Flash;

    const WORD_SIZE: u32 = 4;
    const SECTOR_SIZE: u32 = 4 << 10;
    const NUM_SECTORS: u32 = 3;
    const FLASH_SIZE: u32 = SECTOR_SIZE * NUM_SECTORS;
    const MAX_OFFSET: u32 = SECTOR_SIZE * 1;
    const MAX_LENGTH: u32 = SECTOR_SIZE * 2;

    #[repr(C, align(4))]
    struct TestBuffer {
        data: [u8; FLASH_SIZE as _],
    }

    impl TestBuffer {
        const fn seq() -> Self {
            let mut data = [0u8; FLASH_SIZE as _];
            let mut index = 0;
            while index < FLASH_SIZE {
                data[index as usize] = (index & 0xff) as u8;
                index += 1;
            }
            Self { data }
        }
    }

    impl Default for TestBuffer {
        fn default() -> Self {
            Self {
                data: [0u8; FLASH_SIZE as usize],
            }
        }
    }

    #[cfg(not(miri))]
    fn range_gen<const ALIGN: u32, const MAX_OFF: u32, const MAX_LEN: u32>(
        aligned: Option<bool>,
    ) -> impl Iterator<Item = (u32, u32)> {
        (0..=MAX_OFF).flat_map(move |off| {
            (0..=MAX_LEN - off)
                .filter(move |len| {
                    aligned
                        .map(|aligned| aligned == (off % ALIGN == 0 && len % ALIGN == 0))
                        .unwrap_or(true)
                })
                .map(move |len| (off, len))
        })
    }

    #[cfg(miri)]
    fn range_gen<const ALIGN: u32, const MAX_OFF: u32, const MAX_LEN: u32>(
        aligned: Option<bool>,
    ) -> impl Iterator<Item = (u32, u32)> {
        // MIRI is very slow - just use a couple of combinations
        match aligned {
            Some(true) => vec![(0, 4), (0, 8), (0, 16), (0, 32), (0, 1024)],
            Some(false) => vec![(3, 7), (11, 11)],
            None => vec![
                (0, 4),
                (0, 8),
                (0, 16),
                (0, 32),
                (0, 1024),
                (3, 7),
                (11, 11),
                (0, 4098),
            ],
        }
        .into_iter()
    }

    #[test]
    #[cfg(not(feature = "bytewise-read"))]
    fn aligned_read() {
        let mut flash = FlashStorage::new(Flash::new());
        flash.capacity = 4 * 4096;
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &src.data).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(Some(true)) {
            flash.read(off, &mut data.data[..len as usize]).unwrap();
            assert_eq!(
                data.data[..len as usize],
                src.data[off as usize..][..len as usize]
            );
        }
    }

    #[test]
    #[cfg(not(feature = "bytewise-read"))]
    fn not_aligned_read_aligned_buffer() {
        let mut flash = FlashStorage::new(Flash::new());
        flash.capacity = 4 * 4096;
        let mut data = TestBuffer::default();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(Some(false)) {
            flash.read(off, &mut data.data[..len as usize]).unwrap_err();
        }
    }

    #[test]
    #[cfg(not(feature = "bytewise-read"))]
    fn aligned_read_not_aligned_buffer() {
        let mut flash = FlashStorage::new(Flash::new());
        flash.capacity = 4 * 4096;
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &src.data).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(Some(true)) {
            flash
                .read(off, &mut data.data[1..][..len as usize])
                .unwrap();
            assert_eq!(
                data.data[1..][..len as usize],
                src.data[off as usize..][..len as usize]
            );
        }
    }

    #[test]
    #[cfg(feature = "bytewise-read")]
    fn bytewise_read_aligned_buffer() {
        let mut flash = FlashStorage::new(Flash::new());

        flash.capacity = 4 * 4096;
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &src.data).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(None) {
            flash.read(off, &mut data.data[..len as usize]).unwrap();
            assert_eq!(
                data.data[..len as usize],
                src.data[off as usize..][..len as usize]
            );
        }
    }

    #[test]
    #[cfg(feature = "bytewise-read")]
    fn bytewise_read_not_aligned_buffer() {
        let mut flash = FlashStorage::new(Flash::new());

        flash.capacity = 4 * 4096;
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &src.data).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(None) {
            flash
                .read(off, &mut data.data[1..][..len as usize])
                .unwrap();
            assert_eq!(
                data.data[1..][..len as usize],
                src.data[off as usize..][..len as usize]
            );
        }
    }

    #[test]
    fn write_not_aligned_buffer() {
        let mut flash = FlashStorage::new(Flash::new());
        flash.capacity = 4 * 4096;
        let mut read_data = TestBuffer::default();
        let write_data = TestBuffer::seq();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &write_data.data[1..129]).unwrap();

        flash.read(0, &mut read_data.data[..128]).unwrap();

        assert_eq!(&read_data.data[..128], &write_data.data[1..129]);
    }
}

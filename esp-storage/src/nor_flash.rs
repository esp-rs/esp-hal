use core::mem::MaybeUninit;

use embedded_storage::nor_flash::{
    ErrorType,
    NorFlash,
    NorFlashError,
    NorFlashErrorKind,
    ReadNorFlash,
};

use crate::{FlashSectorBuffer, FlashStorage, FlashStorageError};

#[cfg(feature = "bytewise-read")]
#[repr(C, align(4))]
struct FlashWordBuffer {
    // NOTE: Ensure that no unaligned fields are added above `data` to maintain its required
    // alignment
    data: [u8; FlashStorage::WORD_SIZE as usize],
}

#[cfg(feature = "bytewise-read")]
impl core::ops::Deref for FlashWordBuffer {
    type Target = [u8; FlashStorage::WORD_SIZE as usize];

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

#[cfg(feature = "bytewise-read")]
impl core::ops::DerefMut for FlashWordBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

impl FlashStorage {
    #[inline(always)]
    fn is_word_aligned(bytes: &[u8]) -> bool {
        // TODO: Use is_aligned_to when stabilized (see `pointer_is_aligned`)
        (unsafe { bytes.as_ptr().offset_from(core::ptr::null()) }) % Self::WORD_SIZE as isize == 0
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

impl ErrorType for FlashStorage {
    type Error = FlashStorageError;
}

impl ReadNorFlash for FlashStorage {
    #[cfg(not(feature = "bytewise-read"))]
    const READ_SIZE: usize = Self::WORD_SIZE as _;

    #[cfg(feature = "bytewise-read")]
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.check_alignment::<{ Self::READ_SIZE as _ }>(offset, bytes.len())?;
        self.check_bounds(offset, bytes.len())?;

        #[cfg(feature = "bytewise-read")]
        let (offset, bytes) = {
            let byte_offset = (offset % Self::WORD_SIZE) as usize;
            if byte_offset > 0 {
                let mut word_buffer = MaybeUninit::<FlashWordBuffer>::uninit();
                let word_buffer = unsafe { word_buffer.assume_init_mut() };

                let offset = offset - byte_offset as u32;
                let length = bytes.len().min(word_buffer.len() - byte_offset);

                self.internal_read(offset, &mut word_buffer[..])?;
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
                self.internal_read(offset, chunk)?;

                #[cfg(feature = "bytewise-read")]
                {
                    let length = chunk.len();
                    let byte_length = length % Self::WORD_SIZE as usize;
                    let length = length - byte_length;

                    self.internal_read(offset, &mut chunk[..length])?;

                    // Read not aligned rest of data
                    if byte_length > 0 {
                        let mut word_buffer = MaybeUninit::<FlashWordBuffer>::uninit();
                        let word_buffer = unsafe { word_buffer.assume_init_mut() };

                        self.internal_read(offset + length as u32, &mut word_buffer[..])?;
                        chunk[length..].copy_from_slice(&word_buffer[..byte_length]);
                    }
                }
            }
        } else {
            // Bytes buffer isn't word-aligned so we might read only via aligned buffer
            let mut buffer = MaybeUninit::<FlashSectorBuffer>::uninit();
            let buffer = unsafe { buffer.assume_init_mut() };

            for (offset, chunk) in (offset..)
                .step_by(Self::SECTOR_SIZE as _)
                .zip(bytes.chunks_mut(Self::SECTOR_SIZE as _))
            {
                // Read to temporary buffer first (chunk length is aligned)
                #[cfg(not(feature = "bytewise-read"))]
                self.internal_read(offset, &mut buffer[..chunk.len()])?;

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

                    self.internal_read(offset, &mut buffer[..length])?;
                }

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

impl NorFlash for FlashStorage {
    const WRITE_SIZE: usize = Self::WORD_SIZE as _;
    const ERASE_SIZE: usize = Self::SECTOR_SIZE as _;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.check_alignment::<{ Self::WORD_SIZE }>(offset, bytes.len())?;
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
            let mut buffer = MaybeUninit::<FlashSectorBuffer>::uninit();
            let buffer = unsafe { buffer.assume_init_mut() };

            for (offset, chunk) in (offset..)
                .step_by(Self::SECTOR_SIZE as _)
                .zip(bytes.chunks(Self::SECTOR_SIZE as _))
            {
                // Copy to temporary buffer first
                buffer[..chunk.len()].copy_from_slice(chunk);
                // Write from temporary buffer
                self.internal_write(offset, &buffer[..chunk.len()])?;
            }
        }

        Ok(())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let len = (to - from) as _;
        self.check_alignment::<{ Self::SECTOR_SIZE }>(from, len)?;
        self.check_bounds(from, len)?;

        for sector in from / Self::SECTOR_SIZE..to / Self::SECTOR_SIZE {
            self.internal_erase(sector)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod test {
    use core::mem::MaybeUninit;

    use super::*;

    const WORD_SIZE: u32 = 4;
    const SECTOR_SIZE: u32 = 4 << 10;
    const NUM_SECTORS: u32 = 3;
    const FLASH_SIZE: u32 = SECTOR_SIZE * NUM_SECTORS;
    const MAX_OFFSET: u32 = SECTOR_SIZE * 1;
    const MAX_LENGTH: u32 = SECTOR_SIZE * 2;

    #[repr(C, align(4))]
    struct TestBuffer {
        // NOTE: Ensure that no unaligned fields are added above `data` to maintain its required
        // alignment
        data: MaybeUninit<[u8; FLASH_SIZE as _]>,
    }

    impl TestBuffer {
        const fn seq() -> Self {
            let mut data = [0u8; FLASH_SIZE as _];
            let mut index = 0;
            while index < FLASH_SIZE {
                data[index as usize] = (index & 0xff) as u8;
                index += 1;
            }
            Self {
                data: MaybeUninit::new(data),
            }
        }
    }

    impl Default for TestBuffer {
        fn default() -> Self {
            Self {
                data: MaybeUninit::uninit(),
            }
        }
    }

    impl Deref for TestBuffer {
        type Target = [u8; FLASH_SIZE as usize];

        fn deref(&self) -> &Self::Target {
            unsafe { self.data.assume_init_ref() }
        }
    }

    impl DerefMut for TestBuffer {
        fn deref_mut(&mut self) -> &mut Self::Target {
            unsafe { self.data.assume_init_mut() }
        }
    }

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

    #[test]
    #[cfg(not(feature = "bytewise-read"))]
    fn aligned_read() {
        let mut flash = FlashStorage::new();
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &*src).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(Some(true)) {
            flash.read(off, &mut data[..len as usize]).unwrap();
            assert_eq!(data[..len as usize], src[off as usize..][..len as usize]);
        }
    }

    #[test]
    #[cfg(not(feature = "bytewise-read"))]
    fn not_aligned_read_aligned_buffer() {
        let mut flash = FlashStorage::new();
        let mut data = TestBuffer::default();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(Some(false)) {
            flash.read(off, &mut data[..len as usize]).unwrap_err();
        }
    }

    #[test]
    #[cfg(not(feature = "bytewise-read"))]
    fn aligned_read_not_aligned_buffer() {
        let mut flash = FlashStorage::new();
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &*src).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(Some(true)) {
            flash.read(off, &mut data[1..][..len as usize]).unwrap();
            assert_eq!(
                data[1..][..len as usize],
                src[off as usize..][..len as usize]
            );
        }
    }

    #[test]
    #[cfg(feature = "bytewise-read")]
    fn bytewise_read_aligned_buffer() {
        let mut flash = FlashStorage::new();
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &*src).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(None) {
            flash.read(off, &mut data[..len as usize]).unwrap();
            assert_eq!(data[..len as usize], src[off as usize..][..len as usize]);
        }
    }

    #[test]
    #[cfg(feature = "bytewise-read")]
    fn bytewise_read_not_aligned_buffer() {
        let mut flash = FlashStorage::new();
        let src = TestBuffer::seq();
        let mut data = TestBuffer::default();

        flash.erase(0, FLASH_SIZE).unwrap();
        flash.write(0, &*src).unwrap();

        for (off, len) in range_gen::<WORD_SIZE, MAX_OFFSET, MAX_LENGTH>(None) {
            flash.read(off, &mut data[1..][..len as usize]).unwrap();
            assert_eq!(
                data[1..][..len as usize],
                src[off as usize..][..len as usize]
            );
        }
    }
}

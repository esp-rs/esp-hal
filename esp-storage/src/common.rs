use core::{mem::MaybeUninit, sync::atomic::Ordering};

use portable_atomic::AtomicBool;

use crate::chip_specific;
#[cfg(multi_core)]
use crate::multi_core::MultiCoreStrategy;

// This flag ensures the singleton can only be created once.
static IS_TAKEN: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Flash storage error.
pub enum FlashStorageError {
    /// I/O error.
    IoError,
    /// I/O operation timed out.
    IoTimeout,
    /// Flash could not be unlocked for writing.
    CantUnlock,
    /// Address or length not aligned to required boundary.
    NotAligned,
    /// Address or length out of bounds.
    OutOfBounds,
    /// Cannot write to flash as more than one core is running.
    /// Either manually suspend the other core, or use one of the available strategies:
    /// * [`FlashStorage::multicore_auto_park`]
    /// * [`FlashStorage::multicore_ignore`]
    #[cfg(multi_core)]
    OtherCoreRunning,
    /// Other error with the given error code.
    Other(i32),
}

#[inline(always)]
/// Check return code from flash operations.
pub fn check_rc(rc: i32) -> Result<(), FlashStorageError> {
    match rc {
        0 => Ok(()),
        1 => Err(FlashStorageError::IoError),
        2 => Err(FlashStorageError::IoTimeout),
        _ => Err(FlashStorageError::Other(rc)),
    }
}
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// A zero-sized helper type that ensures only one instance of FlashStorage is created.
pub struct FlashSingleton {
    _private: (),
}

impl FlashSingleton {
    /// Takes the unique singleton token.
    ///
    /// Returns `Some(token)` on the first call, and `None` on all subsequent calls.
    pub fn take() -> Option<Self> {
        let already_taken = IS_TAKEN.fetch_or(true, Ordering::Acquire);
        if !already_taken {
            // If the exchange succeeded, the original value was `false`.
            // This is the first call, so we can safely return the token.
            Some(Self { _private: () })
        } else {
            // The flag was already `true`. Someone else has the token.
            None
        }
    }

    #[cfg(test)]
    pub(crate) fn reset_for_test() {
        IS_TAKEN.store(false, core::sync::atomic::Ordering::Relaxed);
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Flash storage abstraction.
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
    /// Flash word size in bytes.
    pub const WORD_SIZE: u32 = 4;
    /// Flash sector size in bytes.
    pub const SECTOR_SIZE: u32 = 4096;

    /// Create a new flash storage instance.
    pub fn new(singleton: FlashSingleton) -> FlashStorage {
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

#[cfg(test)]
mod tests {
    use crate::FlashSingleton;
    #[test]
    fn test_singleton_behavior() {
        FlashSingleton::reset_for_test();

        // First call should succeed
        let token1 = FlashSingleton::take();
        assert!(token1.is_some());

        // Second call should fail
        let token2 = FlashSingleton::take();
        assert!(token2.is_none());

        // Third call should also fail
        let token3 = FlashSingleton::take();
        assert!(token3.is_none());
    }

    #[test]
    #[should_panic(expected = "This should panic")]
    fn test_expect_panics() {
        let _token1 = FlashSingleton::take().expect("First should work");
        let _token2 = FlashSingleton::take().expect("This should panic");
    }
}

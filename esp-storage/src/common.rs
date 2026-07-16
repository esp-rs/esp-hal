use core::mem::MaybeUninit;

#[cfg(multi_core)]
use esp_hal::peripherals::CPU_CTRL;
#[cfg(not(feature = "emulation"))]
pub use esp_hal::peripherals::FLASH as Flash;
#[cfg(multi_core)]
use esp_hal::system::Cpu;
#[cfg(multi_core)]
use esp_hal::system::CpuControl;
#[cfg(multi_core)]
use esp_hal::system::is_running;

use crate::chip_specific;
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
    /// Operation not supported (e.g. no free MMU entry).
    NotSupported,
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

#[cfg(feature = "emulation")]
#[derive(Debug)]
pub struct Flash<'d> {
    _phantom: core::marker::PhantomData<&'d ()>,
}

#[cfg(feature = "emulation")]
impl<'d> Flash<'d> {
    pub fn new() -> Self {
        Flash {
            _phantom: core::marker::PhantomData,
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Flash storage abstraction.
pub struct FlashStorage<'d> {
    pub(crate) capacity: usize,
    unlocked: bool,
    pub(crate) multi_core_strategy: MultiCoreStrategy,
    _flash: Flash<'d>,
}

impl<'d> FlashStorage<'d> {
    /// Flash word size in bytes.
    pub const WORD_SIZE: u32 = 4;
    /// Flash sector size in bytes.
    pub const SECTOR_SIZE: u32 = 4096;
    /// Flash block size in bytes.
    pub const BLOCK_SIZE: u32 = 65536;

    /// Create a new flash storage instance.
    ///
    /// # Panics
    ///
    /// Panics if called more than once.
    pub fn new(flash: Flash<'d>) -> Self {
        Self {
            capacity: chip_specific::get_flash_size() as usize,
            unlocked: false,
            multi_core_strategy: cfg_select!(
                multi_core => MultiCoreStrategy::Error,
                _ => MultiCoreStrategy::Ignore
            ),
            _flash: flash,
        }
    }

    #[inline(always)]
    pub(crate) fn check_alignment<const ALIGN: u32>(
        &self,
        offset: u32,
        length: usize,
    ) -> Result<(), FlashStorageError> {
        let offset = offset as usize;
        if !offset.is_multiple_of(ALIGN as usize) || !length.is_multiple_of(ALIGN as usize) {
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

    pub(crate) fn internal_erase_sector(&mut self, sector: u32) -> Result<(), FlashStorageError> {
        self.multi_core_strategy.with(|| {
            self.unlock_once()?;
            check_rc(chip_specific::spiflash_erase_sector(sector))
        })
    }

    pub(crate) fn internal_erase_block(&mut self, block: u32) -> Result<(), FlashStorageError> {
        self.multi_core_strategy.with(|| {
            self.unlock_once()?;
            check_rc(chip_specific::spiflash_erase_block(block))
        })
    }

    pub(crate) fn internal_write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        self.multi_core_strategy.with(|| {
            self.unlock_once()?;
            check_rc(chip_specific::spiflash_write(
                offset,
                bytes.as_ptr() as *const u32,
                bytes.len() as u32,
            ))
        })
    }

    pub(crate) fn internal_read_encrypted(
        &mut self,
        offset: u32,
        bytes: &mut [MaybeUninit<u8>],
    ) -> Result<(), FlashStorageError> {
        // SAFETY: `read_flash_encrypted` fully initializes every byte in `bytes`.
        let initialized = unsafe {
            core::slice::from_raw_parts_mut(bytes.as_mut_ptr().cast::<u8>(), bytes.len())
        };
        chip_specific::read_flash_encrypted(offset, initialized)
    }

    pub(crate) fn internal_write_encrypted(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        self.multi_core_strategy.with(|| {
            check_rc(chip_specific::spiflash_write_encrypted(
                offset,
                bytes.as_ptr() as *mut u32,
                bytes.len() as u32,
            ))
        })
    }
}

/// Strategy to use on a multi core system where writing to the flash needs exclusive access from
/// one core.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum MultiCoreStrategy {
    /// Flash writes simply fail if the second core is active while attempting a write (default
    /// behavior).
    #[cfg(multi_core)]
    Error,

    /// Auto park the other core before writing. Un-park it when writing is complete.
    #[cfg(multi_core)]
    AutoPark,

    /// Ignore that the other core is active.
    /// This is useful if the second core is known to not fetch instructions from the flash for the
    /// duration of the write. This is unsafe to use.
    Ignore,
}

#[cfg(multi_core)]
impl<'d> FlashStorage<'d> {
    /// Enable auto parking of the second core before writing to flash.
    /// The other core will be automatically un-parked when the write is complete.
    pub fn multicore_auto_park(mut self) -> FlashStorage<'d> {
        self.multi_core_strategy = MultiCoreStrategy::AutoPark;
        self
    }

    /// Do not check if the second core is active before writing to flash.
    ///
    /// # Safety
    /// Only enable this if you are sure that the second core is not fetching instructions from the
    /// flash during the write.
    pub unsafe fn multicore_ignore(mut self) -> FlashStorage<'d> {
        self.multi_core_strategy = MultiCoreStrategy::Ignore;
        self
    }
}

impl MultiCoreStrategy {
    /// Perform checks/Prepare for a flash write according to the current strategy.
    ///
    /// # Returns
    /// * `True` if the other core needs to be un-parked by post_write
    /// * `False` otherwise
    pub(crate) fn pre_write(&self) -> Result<bool, FlashStorageError> {
        match self {
            #[cfg(multi_core)]
            MultiCoreStrategy::Error => {
                for other_cpu in Cpu::other() {
                    if is_running(other_cpu) {
                        return Err(FlashStorageError::OtherCoreRunning);
                    }
                }
                Ok(false)
            }

            #[cfg(multi_core)]
            MultiCoreStrategy::AutoPark => {
                let mut cpu_ctrl = CpuControl::new(unsafe { CPU_CTRL::steal() });
                for other_cpu in Cpu::other() {
                    if is_running(other_cpu) {
                        unsafe { cpu_ctrl.park_core(other_cpu) };
                        return Ok(true);
                    }
                }
                Ok(false)
            }

            MultiCoreStrategy::Ignore => Ok(false),
        }
    }

    /// Perform post-write actions.
    ///
    /// # Returns
    /// * `True` if the other core needs to be un-parked by post_write
    /// * `False` otherwise
    pub(crate) fn post_write(&self, unpark: bool) {
        cfg_select! {
            multi_core => {
                let mut cpu_ctrl = CpuControl::new(unsafe { CPU_CTRL::steal() });
                if let MultiCoreStrategy::AutoPark = self
                    && unpark
                {
                    for other_cpu in Cpu::other() {
                        cpu_ctrl.unpark_core(other_cpu);
                    }
                }
            }
            _ => {
                let _ = unpark;
            }
        }
    }

    /// Run a flash write operation, handling multi-core synchronization.
    ///
    /// # Returns
    /// * `Ok` with the result of the operation
    /// * `Err` if the operation fails
    pub(crate) fn with<R>(
        self,
        f: impl FnOnce() -> Result<R, FlashStorageError>,
    ) -> Result<R, FlashStorageError> {
        let unpark = self.pre_write()?;

        let result = f();

        self.post_write(unpark);
        result
    }
}

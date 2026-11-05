//! Memory that the backup DMA uses to retain a radio while the modem has no power.

use core::{mem::MaybeUninit, ptr};

use docsplay::Display;
use portable_atomic::{AtomicPtr, Ordering};
use static_cell::ConstStaticCell;

const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
const MEM_END: usize = property!("sleep.cpu_retention_mem_end");

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub struct RetentionStorage<const SIZE: usize> {
    inner: ConstStaticCell<RetentionMemory<SIZE>>,
}

impl<const SIZE: usize> RetentionStorage<SIZE> {
    /// Creates storage for the retention memory.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            inner: ConstStaticCell::new(RetentionMemory {
                storage: MaybeUninit::uninit(),
            }),
        }
    }

    /// Takes the reference to the retention memory.
    ///
    /// # Panics
    ///
    /// Panics if the reference is taken already.
    #[instability::unstable]
    pub fn take(&'static self) -> &'static mut RetentionMemory<SIZE> {
        self.inner.take()
    }
}

#[instability::unstable]
impl<const SIZE: usize> Default for RetentionStorage<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: `ConstStaticCell` holds a `MaybeUninit` buffer and an `AtomicBool`, and both are valid
// when zeroed.
unsafe impl<const SIZE: usize> bytemuck::Zeroable for RetentionStorage<SIZE> {}

/// Memory that a radio uses to retain its state while the modem has no power.
///
/// The backup DMA writes this memory, so the type gives no way to read or write its contents.
///
/// The memory must be in internal RAM that the backup DMA can reach. The
/// [`#[ram(reclaimed)]`][esp_hal::ram] attribute places a static inside that range.
#[repr(align(4))]
#[instability::unstable]
pub struct RetentionMemory<const SIZE: usize> {
    storage: MaybeUninit<[u8; SIZE]>,
}

/// The reason why a radio refused the retention memory.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum RetentionMemoryError {
    /// The memory is outside the range that the backup DMA can reach.
    OutOfRange,
    /// The radio has retention memory already.
    AlreadyInstalled,
}

#[instability::unstable]
impl core::error::Error for RetentionMemoryError {}

/// Holds the retention memory that one radio gets from its `install_retention_memory`.
pub(crate) struct RetentionSlot<const SIZE: usize> {
    base: AtomicPtr<u8>,
}

impl<const SIZE: usize> RetentionSlot<SIZE> {
    pub(crate) const fn new() -> Self {
        Self {
            base: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Keeps the memory for the life of the program.
    pub(crate) fn install(
        &self,
        memory: &'static mut RetentionMemory<SIZE>,
    ) -> Result<(), RetentionMemoryError> {
        let memory = memory.storage.as_mut_ptr().cast::<u8>();

        let start = memory as usize;
        if start < MEM_START || start + SIZE > MEM_END {
            return Err(RetentionMemoryError::OutOfRange);
        }

        self.base
            .compare_exchange(ptr::null_mut(), memory, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| RetentionMemoryError::AlreadyInstalled)
    }

    /// Returns the start of the memory, or null if no memory is installed.
    pub(crate) fn base(&self) -> *mut u8 {
        self.base.load(Ordering::Acquire)
    }

    pub(crate) fn is_installed(&self) -> bool {
        !self.base().is_null()
    }
}

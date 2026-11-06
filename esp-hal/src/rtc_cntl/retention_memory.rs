//! Memory that the hardware writes while a power domain has no power, and its one-shot install.

use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
    ptr::{self, NonNull},
};

use portable_atomic::{AtomicPtr, Ordering};
use static_cell::ConstStaticCell;

#[cfg(not(cpu_retention = "software"))]
const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
#[cfg(not(cpu_retention = "software"))]
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
                storage: UnsafeCell::new(MaybeUninit::uninit()),
            }),
        }
    }

    /// Takes the reference to the retention memory.
    ///
    /// Can only be called once.
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

// SAFETY: ConstStaticCell is a `MaybeUninit` buffer, and an AtomicBool strapped together,
// both are safe to be zero-initialized.
unsafe impl<const SIZE: usize> bytemuck::Zeroable for RetentionStorage<SIZE> {}

/// Memory that the hardware writes while a power domain has no power.
///
/// The hardware writes this memory, so the type gives no way to read or write its contents through
/// safe Rust.
///
/// The memory must be in internal RAM that stays powered across light sleep. The
/// [`#[ram(reclaimed)]`][crate::ram] attribute places a static inside the range that the
/// retention DMA reaches, on chips that use the retention DMA.
#[repr(align(16))]
#[instability::unstable]
pub struct RetentionMemory<const SIZE: usize> {
    storage: UnsafeCell<MaybeUninit<[u8; SIZE]>>,
}

/// The reason why a driver refused the retention memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum RetentionMemoryError {
    /// The memory is not inside the range that the retention DMA reaches.
    #[cfg(not(cpu_retention = "software"))]
    OutOfRange,
    /// The driver has retention memory already.
    AlreadyInstalled,
}

#[instability::unstable]
impl core::error::Error for RetentionMemoryError {}

#[instability::unstable]
impl core::fmt::Display for RetentionMemoryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(not(cpu_retention = "software"))]
            Self::OutOfRange => write!(
                f,
                "the memory is outside the retention memory range (0x{MEM_START:X}..0x{MEM_END:X})"
            ),
            Self::AlreadyInstalled => write!(f, "retention memory is installed already"),
        }
    }
}

/// Holds the [`RetentionMemory`] of one driver for the life of the program.
#[doc(hidden)]
#[instability::unstable]
pub struct RetentionSlot<const SIZE: usize> {
    memory: AtomicPtr<u8>,
}

impl<const SIZE: usize> RetentionSlot<SIZE> {
    /// Creates a slot without memory.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            memory: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Keeps the memory for the life of the program.
    ///
    /// # Errors
    ///
    /// Returns [`RetentionMemoryError`] if the slot holds memory already, or if the memory is not
    /// inside the range that the retention DMA reaches.
    #[instability::unstable]
    pub fn install(
        &self,
        memory: &'static mut RetentionMemory<SIZE>,
    ) -> Result<(), RetentionMemoryError> {
        let memory = memory.storage.get().cast::<u8>();

        #[cfg(not(cpu_retention = "software"))]
        {
            let start = memory as usize;
            if start < MEM_START || start + SIZE > MEM_END {
                return Err(RetentionMemoryError::OutOfRange);
            }
        }

        self.memory
            .compare_exchange(ptr::null_mut(), memory, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| RetentionMemoryError::AlreadyInstalled)
    }

    /// Returns the start of the memory, if the slot holds it.
    #[instability::unstable]
    pub fn get(&self) -> Option<NonNull<u8>> {
        NonNull::new(self.memory.load(Ordering::Acquire))
    }
}

#[instability::unstable]
impl<const SIZE: usize> Default for RetentionSlot<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

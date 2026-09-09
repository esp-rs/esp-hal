//! Shared CPU retention buffer types and the one-shot install API.

use core::{cell::UnsafeCell, mem::MaybeUninit, ptr};

use portable_atomic::{AtomicPtr, Ordering};
use static_cell::ConstStaticCell;

use crate::rtc_cntl::sleep::LowPower;

const BUFFER_SIZE: usize = property!("sleep.cpu_retention_mem_size");
const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
const MEM_END: usize = property!("sleep.cpu_retention_mem_end");

/// `repr(align)` takes a literal, so the buffer takes the strictest alignment of every chip. The
/// alignment then needs no check at run time.
const _: () = ::core::assert!(property!("sleep.cpu_retention_mem_align") <= 16);

static INSTALLED: AtomicPtr<CpuRetentionMemory> = AtomicPtr::new(ptr::null_mut());

/// Memory that light sleep uses to retain the CPU domain.
///
/// This storage must be placed to `#[ram(reclaimed, unstable(zeroed))]`, then
/// a static mutable reference to [`CpuRetentionMemory`] can be taken out of it.
#[instability::unstable]
pub struct CpuRetentionStorage {
    inner: ConstStaticCell<CpuRetentionMemory>,
}

impl CpuRetentionStorage {
    /// Creates storage for the retention memory.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            inner: ConstStaticCell::new(CpuRetentionMemory {
                storage: UnsafeCell::new(MaybeUninit::uninit()),
            }),
        }
    }

    /// Takes the reference to the retention memory.
    ///
    /// Can only be called once.
    #[instability::unstable]
    pub fn take(&'static self) -> &'static mut CpuRetentionMemory {
        self.inner.take()
    }
}

#[instability::unstable]
impl Default for CpuRetentionStorage {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: ConstStaticCell is a `MaybeUninit` buffer, and an AtomicBool strapped together,
// both are safe to be zero-initialized.
unsafe impl bytemuck::Zeroable for CpuRetentionStorage {}

/// Memory that light sleep uses to retain the CPU domain.
///
/// The sleep path writes this memory while the CPU domain has no power, so the buffer gives no way
/// to read or write its contents through safe Rust.
///
/// Install the buffer with [`LowPower::install_cpu_retention_memory`]. The buffer must live in
/// internal RAM that stays powered across light sleep. The [`#[ram(reclaimed)]`][crate::ram]
/// attribute places a static inside the reachable range on chips that use the retention DMA.
#[repr(align(16))]
#[instability::unstable]
pub struct CpuRetentionMemory {
    storage: UnsafeCell<MaybeUninit<[u8; BUFFER_SIZE]>>,
}

impl CpuRetentionMemory {
    pub(crate) fn as_mut_ptr(&self) -> *mut u8 {
        self.storage.get().cast()
    }
}

/// The reason why [`LowPower::install_cpu_retention_memory`] refused a buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum CpuRetentionMemoryError {
    /// The buffer is not inside the range that retention needs.
    OutOfRange,
    /// The driver holds a retention buffer already.
    AlreadyInstalled,
}

#[instability::unstable]
impl core::error::Error for CpuRetentionMemoryError {}

#[instability::unstable]
impl core::fmt::Display for CpuRetentionMemoryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::OutOfRange => write!(
                f,
                "the buffer is outside the retention memory range (0x{MEM_START:X}..0x{MEM_END:X})"
            ),
            Self::AlreadyInstalled => write!(f, "a retention buffer is installed already"),
        }
    }
}

impl LowPower<'_> {
    #[procmacros::doc_replace]
    /// Gives light sleep the memory that it uses to retain the CPU domain.
    ///
    /// Light sleep powers the CPU domain down only after it has this memory. The memory stays with
    /// the driver for the life of the program, and a second call is an error, because the sleep
    /// path can hold one buffer only.
    ///
    /// # Errors
    ///
    /// Returns [`CpuRetentionMemoryError`] if the driver holds a buffer already, or if the buffer
    /// is not inside the range that retention needs. The
    /// [`#[ram(reclaimed)]`][crate::ram] attribute places a static inside that range on DMA
    /// chips.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rtc_cntl::{CpuRetentionStorage, sleep::LowPower};
    ///
    /// #[ram(reclaimed, unstable(zeroed))]
    /// static RETENTION: CpuRetentionStorage = CpuRetentionStorage::new();
    ///
    /// let mut lpwr = LowPower::new(peripherals.LPWR);
    /// lpwr.install_cpu_retention_memory(RETENTION.take())?;
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn install_cpu_retention_memory(
        &mut self,
        memory: &'static mut CpuRetentionMemory,
    ) -> Result<(), CpuRetentionMemoryError> {
        let memory = ptr::from_mut(memory);
        let start = memory as usize;

        if start < MEM_START || start + BUFFER_SIZE > MEM_END {
            return Err(CpuRetentionMemoryError::OutOfRange);
        }

        INSTALLED
            .compare_exchange(ptr::null_mut(), memory, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| ())
            .map_err(|_| CpuRetentionMemoryError::AlreadyInstalled)
    }
}

pub(crate) fn installed_buffer_ptr() -> Option<*mut u8> {
    // SAFETY: `INSTALLED` holds a pointer that `install_cpu_retention_memory` took as a
    // `&'static mut`, so the memory outlives the read.
    let memory = unsafe { INSTALLED.load(Ordering::Acquire).as_ref()? };

    Some(memory.as_mut_ptr())
}

pub(crate) const fn buffer_size() -> usize {
    BUFFER_SIZE
}

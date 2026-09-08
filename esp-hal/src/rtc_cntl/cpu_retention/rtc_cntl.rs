//! CPU retention through the RTC_CNTL retention DMA.

use core::{cell::UnsafeCell, mem::MaybeUninit, ptr};

use portable_atomic::{AtomicPtr, Ordering};

use crate::rtc_cntl::sleep::LowPower;

const BUFFER_SIZE: usize = property!("sleep.cpu_retention_mem_size");
const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
const MEM_END: usize = property!("sleep.cpu_retention_mem_end");

/// `repr(align)` takes a literal, so the buffer takes the strictest alignment of every chip. The
/// alignment then needs no check at run time.
const _: () = assert!(property!("sleep.cpu_retention_mem_align") <= 16);

static INSTALLED: AtomicPtr<CpuRetentionMemory> = AtomicPtr::new(ptr::null_mut());

/// Memory that light sleep uses to retain the CPU domain.
///
/// The retention DMA writes this memory while the CPU is powered down, so the buffer gives no way
/// to read or write its contents.
///
/// Install the buffer with [`LowPower::install_cpu_retention_memory`]. The retention DMA reaches
/// one range of internal RAM only, which the [`#[ram(unstable(retention))]`][crate::ram] attribute
/// takes care of.
#[repr(align(16))]
#[instability::unstable]
pub struct CpuRetentionMemory {
    storage: UnsafeCell<MaybeUninit<[u8; BUFFER_SIZE]>>,
}

impl CpuRetentionMemory {
    /// Creates the retention memory.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            storage: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    fn as_mut_ptr(&self) -> *mut u8 {
        self.storage.get().cast()
    }
}

#[instability::unstable]
impl Default for CpuRetentionMemory {
    fn default() -> Self {
        Self::new()
    }
}

/// The reason why [`LowPower::install_cpu_retention_memory`] refused a buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum CpuRetentionMemoryError {
    /// The buffer is not inside the range that the retention DMA reaches.
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
    /// is not inside the range that the retention DMA reaches.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::rtc_cntl::{CpuRetentionMemory, sleep::LowPower};
    /// use static_cell::ConstStaticCell;
    ///
    /// static RETENTION: ConstStaticCell<CpuRetentionMemory> =
    ///     ConstStaticCell::new(CpuRetentionMemory::new());
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

//! Memory that the backup DMA uses to retain the BLE MAC and baseband while they have no power.

use core::{mem::MaybeUninit, ptr};

use docsplay::Display;
use esp_sync::NonReentrantMutex;
use portable_atomic::{AtomicPtr, Ordering};
use static_cell::ConstStaticCell;

use crate::sys::c_types::c_void;

// The controller requests three buffers of 1100, 300 and 180 bytes, and frees them together.
const BUFFER_SIZE: usize = 1100 + 300 + 180;

const MEM_START: usize = property!("sleep.cpu_retention_mem_start");
const MEM_END: usize = property!("sleep.cpu_retention_mem_end");

static INSTALLED: AtomicPtr<u8> = AtomicPtr::new(ptr::null_mut());

struct Usage {
    /// Bytes given out from the start of the memory.
    used: usize,
    /// Buffers that the controller did not free yet.
    live: usize,
}

static USAGE: NonReentrantMutex<Usage> = NonReentrantMutex::new(Usage { used: 0, live: 0 });

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub struct RetentionStorage {
    inner: ConstStaticCell<RetentionMemory>,
}

impl RetentionStorage {
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
    pub fn take(&'static self) -> &'static mut RetentionMemory {
        self.inner.take()
    }
}

#[instability::unstable]
impl Default for RetentionStorage {
    fn default() -> Self {
        Self::new()
    }
}

// SAFETY: `ConstStaticCell` holds a `MaybeUninit` buffer and an `AtomicBool`, and both are valid
// when zeroed.
unsafe impl bytemuck::Zeroable for RetentionStorage {}

/// Memory that the BLE controller uses to retain the MAC and baseband while they have no power.
///
/// The backup DMA writes this memory, so the type gives no way to read or write its contents.
///
/// Install the memory with [`install_retention_memory`]. The memory must be in internal RAM that
/// the backup DMA can reach. The [`#[ram(reclaimed)]`][esp_hal::ram] attribute places a static
/// inside that range.
#[repr(align(4))]
#[instability::unstable]
pub struct RetentionMemory {
    storage: MaybeUninit<[u8; BUFFER_SIZE]>,
}

/// The reason why [`install_retention_memory`] refused the memory.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum RetentionMemoryError {
    /// The memory is outside the range that the backup DMA can reach.
    OutOfRange,
    /// The controller has retention memory already.
    AlreadyInstalled,
}

#[instability::unstable]
impl core::error::Error for RetentionMemoryError {}

#[procmacros::doc_replace]
/// Gives the BLE controller the memory that it uses to retain the MAC and baseband.
/// The memory stays with the controller for the life of the program.
///
/// # Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_radio::ble::{RetentionStorage, install_retention_memory};
///
/// #[esp_hal::ram(reclaimed, unstable(zeroed))]
/// static RETENTION: RetentionStorage = RetentionStorage::new();
///
/// install_retention_memory(RETENTION.take())?;
/// # {after_snippet}
/// ```
///
/// # Errors
///
/// Returns [`RetentionMemoryError`] if the controller has retention memory already, or if the
/// memory is outside the range that the backup DMA can reach.
#[instability::unstable]
pub fn install_retention_memory(
    memory: &'static mut RetentionMemory,
) -> Result<(), RetentionMemoryError> {
    let memory = memory.storage.as_mut_ptr().cast::<u8>();

    let start = memory as usize;
    if start < MEM_START || start + BUFFER_SIZE > MEM_END {
        return Err(RetentionMemoryError::OutOfRange);
    }

    INSTALLED
        .compare_exchange(ptr::null_mut(), memory, Ordering::AcqRel, Ordering::Acquire)
        .map(|_| ())
        .map_err(|_| RetentionMemoryError::AlreadyInstalled)
}

/// Returns whether retention memory is installed.
pub(crate) fn is_installed() -> bool {
    !INSTALLED.load(Ordering::Acquire).is_null()
}

/// Allocates a buffer that the backup DMA can reach.
///
/// Returns null if no retention memory is installed, or if the memory has no space left.
pub(crate) extern "C" fn malloc_retention(size: u32) -> *mut c_void {
    let base = INSTALLED.load(Ordering::Acquire);
    if base.is_null() {
        return ptr::null_mut();
    }

    let size = (size as usize).next_multiple_of(4);
    USAGE.with(|usage| {
        if usage.used + size > BUFFER_SIZE {
            return ptr::null_mut();
        }

        let buffer = base.wrapping_add(usage.used);
        usage.used += size;
        usage.live += 1;
        buffer.cast()
    })
}

/// Frees a buffer that [`malloc_retention`] allocated.
///
/// Returns `false` if `ptr` is not in the retention memory.
pub(crate) fn try_free(ptr: *mut c_void) -> bool {
    let base = INSTALLED.load(Ordering::Acquire) as usize;
    if base == 0 || !(base..base + BUFFER_SIZE).contains(&(ptr as usize)) {
        return false;
    }

    USAGE.with(|usage| {
        usage.live -= 1;
        if usage.live == 0 {
            usage.used = 0;
        }
    });
    true
}

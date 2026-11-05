//! Memory that the backup DMA uses to retain the BLE MAC and baseband while they have no power.

use core::ptr;

use esp_sync::NonReentrantMutex;

#[instability::unstable]
pub use crate::retention::RetentionMemoryError;
use crate::{retention::RetentionSlot, sys::c_types::c_void};

// The controller requests three buffers of 1100, 300 and 180 bytes, and frees them together.
const BUFFER_SIZE: usize = 1100 + 300 + 180;

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub type RetentionStorage = crate::retention::RetentionStorage<BUFFER_SIZE>;

/// Memory that the BLE controller uses to retain the MAC and baseband while they have no power.
///
/// Install the memory with [`install_retention_memory`].
#[instability::unstable]
pub type RetentionMemory = crate::retention::RetentionMemory<BUFFER_SIZE>;

static MEMORY: RetentionSlot<BUFFER_SIZE> = RetentionSlot::new();

struct Usage {
    /// Bytes given out from the start of the memory.
    used: usize,
    /// Buffers that the controller did not free yet.
    live: usize,
}

static USAGE: NonReentrantMutex<Usage> = NonReentrantMutex::new(Usage { used: 0, live: 0 });

#[procmacros::doc_replace]
/// Gives the BLE controller the memory that it uses to retain the MAC and baseband.
/// The memory stays with the controller for the life of the program.
///
/// With modem sleep on, the controller then powers the MAC and baseband down while it sleeps.
/// The power-down also takes 768 bytes of internal heap for the PHY state. Install the memory
/// before the controller starts.
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
    MEMORY.install(memory)
}

/// Returns whether retention memory is installed.
pub(crate) fn is_installed() -> bool {
    MEMORY.is_installed()
}

/// Allocates a buffer that the backup DMA can reach.
///
/// Returns null if no retention memory is installed, or if the memory has no space left.
pub(crate) extern "C" fn malloc_retention(size: u32) -> *mut c_void {
    let base = MEMORY.base();
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
    let base = MEMORY.base() as usize;
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

//! Memory that the backup DMA uses to retain the Wi-Fi MAC while it has no power.

use core::ptr;

use portable_atomic::{AtomicBool, Ordering};

use super::WifiError;
#[instability::unstable]
pub use crate::retention::RetentionMemoryError;
use crate::{retention::RetentionSlot, sys::c_types::c_void};

// The blob requests one buffer of this size in `esp_wifi_internal_set_mac_sleep`.
const BUFFER_SIZE: usize = 3280;

/// Storage for [`RetentionMemory`].
///
/// This storage must be placed in `#[ram(reclaimed, unstable(zeroed))]`, then a static mutable
/// reference to [`RetentionMemory`] can be taken out of it.
#[instability::unstable]
pub type RetentionStorage = crate::retention::RetentionStorage<BUFFER_SIZE>;

/// Memory that the Wi-Fi driver uses to retain the MAC while it has no power.
///
/// Install the memory with [`install_retention_memory`].
#[instability::unstable]
pub type RetentionMemory = crate::retention::RetentionMemory<BUFFER_SIZE>;

static MEMORY: RetentionSlot<BUFFER_SIZE> = RetentionSlot::new();
/// Whether the blob holds the memory.
static TAKEN: AtomicBool = AtomicBool::new(false);
/// Whether an allocation of the buffer size gets the memory.
static OFFERED: AtomicBool = AtomicBool::new(false);

#[procmacros::doc_replace]
/// Gives the Wi-Fi driver the memory that it uses to retain the MAC.
/// The memory stays with the driver for the life of the program.
///
/// While the station is in power save, light sleep then powers the MAC and baseband down. The
/// modem domain loses power only when no other radio uses it. The power-down also takes 768 bytes
/// of internal heap for the PHY state. Install the memory before the Wi-Fi controller starts.
///
/// # Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_radio::wifi::{RetentionStorage, install_retention_memory};
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
/// Returns [`RetentionMemoryError`] if the driver has retention memory already, or if the memory
/// is outside the range that the backup DMA can reach.
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

/// Runs `init`, which allocates the MAC buffer, and offers the retention memory to that allocation.
///
/// Returns whether the blob holds the retention memory.
pub(crate) fn offer(init: impl FnOnce() -> Result<(), WifiError>) -> Result<bool, WifiError> {
    OFFERED.store(true, Ordering::Release);
    let result = init();
    OFFERED.store(false, Ordering::Release);
    result.map(|()| TAKEN.load(Ordering::Acquire))
}

/// Allocates the retention memory for the MAC buffer.
///
/// Returns null outside [`offer`], for any other size, and if the memory is not free.
pub(crate) fn malloc(size: usize) -> *mut c_void {
    if size != BUFFER_SIZE || !OFFERED.load(Ordering::Acquire) {
        return ptr::null_mut();
    }

    let base = MEMORY.base();
    if base.is_null() || TAKEN.swap(true, Ordering::AcqRel) {
        return ptr::null_mut();
    }
    base.cast()
}

/// Frees the retention memory.
///
/// Returns `false` if `ptr` is not the retention memory.
pub(crate) fn try_free(ptr: *mut c_void) -> bool {
    let base = MEMORY.base();
    if base.is_null() || ptr.cast::<u8>() != base {
        return false;
    }

    TAKEN.store(false, Ordering::Release);
    true
}

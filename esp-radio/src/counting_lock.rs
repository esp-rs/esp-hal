//! A lock for C code that does not pass the restore state from the acquire to the release.

use esp_sync::{RawMutex, RestoreState};
use portable_atomic::{AtomicU32, Ordering};

/// A [`RawMutex`] that keeps its restore state, and counts nested acquisitions.
///
/// Only the outermost release gives the mutex back.
pub(crate) struct CountingLock {
    lock: &'static RawMutex,
    nesting: AtomicU32,
    token: AtomicU32,
}

impl CountingLock {
    pub(crate) const fn new(lock: &'static RawMutex) -> Self {
        Self {
            lock,
            nesting: AtomicU32::new(0),
            token: AtomicU32::new(0),
        }
    }

    /// Acquires the lock.
    ///
    /// # Safety
    ///
    /// Each acquisition must be paired with a [`Self::release`].
    #[inline(always)]
    pub(crate) unsafe fn acquire(&self) {
        // Take the mutex before the count, so that another core waits for it. A nested
        // acquisition gets a reentry token, which needs no release.
        let token = unsafe { self.lock.acquire() };
        if self.nesting.fetch_add(1, Ordering::Relaxed) == 0 {
            self.token.store(token.inner(), Ordering::Relaxed);
        }
    }

    /// Releases the lock.
    ///
    /// # Safety
    ///
    /// Each release must be paired with an earlier [`Self::acquire`].
    #[inline(always)]
    pub(crate) unsafe fn release(&self) {
        if self.nesting.fetch_sub(1, Ordering::Relaxed) == 1 {
            let token = self.token.load(Ordering::Relaxed);
            unsafe { self.lock.release(RestoreState::new(token)) };
        }
    }

    /// Returns the number of acquisitions that are not released yet.
    #[cfg(all(feature = "ble", bt_controller = "btdm2"))]
    pub(crate) fn nesting(&self) -> u32 {
        self.nesting.load(Ordering::Acquire)
    }
}

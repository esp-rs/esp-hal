//! Asynchronous utilities.
use core::task::Waker;

use embassy_sync::waitqueue::GenericAtomicWaker;
use esp_sync::RawMutex;

/// Utility struct to register and wake a waker.
pub struct AtomicWaker {
    waker: GenericAtomicWaker<RawMutex>,
}

impl AtomicWaker {
    /// Create a new `AtomicWaker`.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            waker: GenericAtomicWaker::new(RawMutex::new()),
        }
    }

    /// Register a waker. Overwrites the previous waker, if any.
    #[inline]
    pub fn register(&self, w: &Waker) {
        self.waker.register(w);
    }

    /// Wake the registered waker, if any.
    #[crate::ram]
    pub fn wake(&self) {
        self.waker.wake();
    }
}

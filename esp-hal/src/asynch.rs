//! Asynchronous utilities.
use core::task::Waker;

use embassy_sync::waitqueue::GenericAtomicWaker;

use crate::sync::RawMutex;

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

    delegate::delegate! {
        to self.waker {
            /// Register a waker. Overwrites the previous waker, if any.
            pub fn register(&self, w: &Waker);
            /// Wake the registered waker, if any.
            pub fn wake(&self);
        }
    }
}

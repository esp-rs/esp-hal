use core::sync::atomic::Ordering;

use portable_atomic::AtomicU32;

// Refcount of 1 is special, indicating that the radio is being initialized or deinitialized. If a
// caller encounters this state, it must spin until the refcount changes.

/// A resource guard that uses a lock-free reference count to track usage.
pub(crate) struct Refcount(AtomicU32);

impl Refcount {
    pub const fn new() -> Self {
        Self(AtomicU32::new(0))
    }

    pub fn increment(&self, on_first: impl FnOnce()) {
        loop {
            let op = self
                .0
                .fetch_update(Ordering::Release, Ordering::Acquire, |old| {
                    if old == 1 { None } else { Some(old + 1) }
                });

            match op {
                Ok(0) => {
                    on_first();
                    self.0.store(2, Ordering::Release);
                    break;
                }
                Ok(_) => break,
                Err(_) => {}
            }
        }
    }

    #[cfg(feature = "wifi")]
    pub fn try_increment<E>(&self, on_first: impl FnOnce() -> Result<(), E>) -> Result<bool, E> {
        loop {
            let op = self
                .0
                .fetch_update(Ordering::Release, Ordering::Acquire, |old| {
                    if old == 1 { None } else { Some(old + 1) }
                });

            match op {
                Ok(0) => {
                    return match on_first() {
                        Ok(()) => {
                            self.0.store(2, Ordering::Release);
                            Ok(true)
                        }
                        Err(e) => {
                            self.0.store(0, Ordering::Release);
                            Err(e)
                        }
                    };
                }
                Ok(_) => return Ok(false),
                Err(_) => {}
            }
        }
    }

    pub fn decrement(&self, on_last: impl FnOnce()) {
        loop {
            let op = self
                .0
                .fetch_update(Ordering::Release, Ordering::Acquire, |old| {
                    if old == 1 { None } else { Some(old - 1) }
                });

            match op {
                Ok(2) => {
                    on_last();
                    self.0.store(0, Ordering::Release);
                    break;
                }
                Ok(_) => break,
                Err(_) => {}
            }
        }
    }
}

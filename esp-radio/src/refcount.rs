use core::{ptr::null_mut, sync::atomic::Ordering};

use esp_radio_rtos_driver::semaphore::{SemaphoreHandle, SemaphoreKind, SemaphorePtr};
use portable_atomic::{AtomicPtr, AtomicU32};

/// Resource guard that handles initialization and deinitalization gracefully
/// using [`esp_radio_rtos_driver`]'s API.
pub(crate) struct Refcount {
    counter: AtomicU32,
    sem: AtomicPtr<()>,
}

impl Drop for Refcount {
    fn drop(&mut self) {
        let sem = self.sem.load(Ordering::Relaxed);
        if let Some(sem) = SemaphorePtr::new(sem) {
            drop(unsafe { SemaphoreHandle::from_ptr(sem) });
        }
    }
}

impl Refcount {
    pub const fn new() -> Self {
        Self {
            counter: AtomicU32::new(0),
            sem: AtomicPtr::new(null_mut()),
        }
    }

    fn use_sem_or_init<T>(&self, f: impl FnOnce(&SemaphoreHandle) -> T) -> T {
        if self.sem.load(Ordering::Relaxed).is_null() {
            core::hint::cold_path();

            let sem = SemaphoreHandle::new(SemaphoreKind::Mutex).leak();

            if self
                .sem
                .compare_exchange(
                    null_mut(),
                    sem.as_ptr(),
                    Ordering::Release,
                    Ordering::Relaxed,
                )
                .is_err()
            {
                core::hint::cold_path();

                drop(unsafe { SemaphoreHandle::from_ptr(sem) });
            }
        }

        let sem = unsafe { SemaphorePtr::new_unchecked(self.sem.load(Ordering::Acquire)) };
        f(unsafe { SemaphoreHandle::ref_from_ptr(&sem) })
    }

    fn lock<T>(&self, f: impl FnOnce() -> T) -> T {
        self.use_sem_or_init(|sem| {
            sem.take(None);
            let ret = f();
            sem.give();
            ret
        })
    }

    pub fn increment(&self, on_first: impl FnOnce()) {
        self.lock(|| {
            if self.counter.fetch_add(1, Ordering::Relaxed) == 0 {
                on_first();
            }
        });
    }

    #[cfg(feature = "wifi")]
    pub fn try_increment<E>(&self, on_first: impl FnOnce() -> Result<(), E>) -> Result<bool, E> {
        self.lock(|| {
            if self.counter.fetch_add(1, Ordering::Relaxed) == 0 {
                on_first()
                    .inspect_err(|_| self.counter.store(0, Ordering::Relaxed))
                    .map(|_| true)
            } else {
                Ok(false)
            }
        })
    }

    pub fn decrement(&self, on_last: impl FnOnce()) {
        self.lock(|| {
            if self.counter.fetch_sub(1, Ordering::Relaxed) == 1 {
                on_last();
            }
        })
    }
}

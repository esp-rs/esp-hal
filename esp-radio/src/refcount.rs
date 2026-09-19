use core::{cell::UnsafeCell, ptr::null_mut, sync::atomic::Ordering};

use esp_radio_rtos_driver::semaphore::{SemaphoreHandle, SemaphoreKind, SemaphorePtr};
use portable_atomic::AtomicPtr;

/// Resource guard that handles initialization and deinitalization gracefully
/// using [`esp_radio_rtos_driver`]'s API.
pub(crate) struct Refcount {
    counter: UnsafeCell<u32>,
    sem: AtomicPtr<()>,
}

unsafe impl Sync for Refcount {}

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
            counter: UnsafeCell::new(0),
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

    fn try_use_sem<T>(&self, f: impl FnOnce(&SemaphoreHandle) -> T) -> Option<T> {
        if self.sem.load(Ordering::Relaxed).is_null() {
            core::hint::cold_path();

            None
        } else {
            let sem = unsafe { SemaphorePtr::new_unchecked(self.sem.load(Ordering::Acquire)) };
            Some(f(unsafe { SemaphoreHandle::ref_from_ptr(&sem) }))
        }
    }

    fn lock<T>(&self, f: impl FnOnce(&mut u32) -> T) -> T {
        self.use_sem_or_init(|sem| {
            sem.take(None);
            let ret = f(unsafe { self.counter.get().as_mut_unchecked() });
            sem.give();
            ret
        })
    }

    fn try_lock<T>(&self, f: impl FnOnce(&mut u32) -> T) -> Option<T> {
        self.try_use_sem(|sem| {
            sem.take(None);
            let ret = f(unsafe { self.counter.get().as_mut_unchecked() });
            sem.give();
            ret
        })
    }

    pub fn increment(&self, on_first: impl FnOnce()) {
        self.lock(|counter| {
            if *counter == 0 {
                on_first();
            }
            *counter = counter.checked_add(1).expect("refcount overflow");
        });
    }

    #[cfg(feature = "wifi")]
    pub fn try_increment<E>(&self, on_first: impl FnOnce() -> Result<(), E>) -> Result<bool, E> {
        self.lock(|counter| {
            let prev = *counter;
            *counter = counter.checked_add(1).expect("refcount overflow");

            if prev == 0 {
                on_first().inspect_err(|_| *counter = 0).map(|_| true)
            } else {
                Ok(false)
            }
        })
    }

    pub fn decrement(&self, on_last: impl FnOnce()) {
        self.try_lock(|counter| {
            if *counter == 0 {
                on_last();
            }
            *counter = counter.checked_sub(1).expect("decrementing count of zero");
        })
        .expect("decrementing before any successful increment")
    }
}

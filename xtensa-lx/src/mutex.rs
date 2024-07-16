//! A series of Mutex's that also implements the `mutex-trait`.

use core::cell::UnsafeCell;

pub use mutex_trait::{self, Mutex};

/// A spinlock and critical section section based mutex.
#[cfg(feature = "spin")]
#[derive(Default)]
pub struct CriticalSectionSpinLockMutex<T> {
    data: spin::Mutex<T>,
}

#[cfg(feature = "spin")]
impl<T> CriticalSectionSpinLockMutex<T> {
    /// Create a new mutex
    pub const fn new(data: T) -> Self {
        CriticalSectionSpinLockMutex {
            data: spin::Mutex::new(data),
        }
    }
}

#[cfg(feature = "spin")]
impl<T> mutex_trait::Mutex for &'_ CriticalSectionSpinLockMutex<T> {
    type Data = T;

    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        crate::interrupt::free(|_| f(&mut (*self.data.lock())))
    }
}

// NOTE A `Mutex` can be used as a channel so the protected data must be `Send`
// to prevent sending non-Sendable stuff (e.g. access tokens) across different
// execution contexts (e.g. interrupts)
#[cfg(feature = "spin")]
unsafe impl<T> Sync for CriticalSectionSpinLockMutex<T> where T: Send {}

/// A Mutex based on critical sections
///
/// # Safety
///
/// **This Mutex is only safe on single-core applications.**
///
/// A `CriticalSection` **is not sufficient** to ensure exclusive access across
/// cores.
#[derive(Default)]
pub struct CriticalSectionMutex<T> {
    data: UnsafeCell<T>,
}

impl<T> CriticalSectionMutex<T> {
    /// Create a new mutex
    pub const fn new(data: T) -> Self {
        CriticalSectionMutex {
            data: UnsafeCell::new(data),
        }
    }
}

impl<T> mutex_trait::Mutex for &'_ CriticalSectionMutex<T> {
    type Data = T;

    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        crate::interrupt::free(|_| f(unsafe { &mut *self.data.get() }))
    }
}

// NOTE A `Mutex` can be used as a channel so the protected data must be `Send`
// to prevent sending non-Sendable stuff (e.g. access tokens) across different
// execution contexts (e.g. interrupts)
unsafe impl<T> Sync for CriticalSectionMutex<T> where T: Send {}

/// A spinlock based mutex.
#[cfg(feature = "spin")]
#[derive(Default)]
pub struct SpinLockMutex<T> {
    data: spin::Mutex<T>,
}

#[cfg(feature = "spin")]
impl<T> SpinLockMutex<T> {
    /// Create a new mutex
    pub const fn new(data: T) -> Self {
        SpinLockMutex {
            data: spin::Mutex::new(data),
        }
    }
}

#[cfg(feature = "spin")]
impl<T> mutex_trait::Mutex for &'_ SpinLockMutex<T> {
    type Data = T;

    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        f(&mut (*self.data.lock()))
    }
}

// NOTE A `Mutex` can be used as a channel so the protected data must be `Send`
// to prevent sending non-Sendable stuff (e.g. access tokens) across different
// execution contexts (e.g. interrupts)
#[cfg(feature = "spin")]
unsafe impl<T> Sync for SpinLockMutex<T> where T: Send {}

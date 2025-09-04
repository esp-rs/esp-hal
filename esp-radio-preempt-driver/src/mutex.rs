//! Mutexes

use core::ptr::NonNull;

/// Pointer to an opaque mutex created by the driver implementation.
pub type MutexPtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_preempt_mutex_create(recursive: bool) -> MutexPtr;
    fn esp_preempt_mutex_delete(mutex: MutexPtr);

    fn esp_preempt_mutex_lock(mutex: MutexPtr, timeout_us: Option<u32>) -> bool;
    fn esp_preempt_mutex_unlock(mutex: MutexPtr) -> bool;
}

pub trait MutexImplementation {
    /// Creates a new mutex instance.
    fn create(recursive: bool) -> MutexPtr;

    /// Deletes a mutex instance.
    ///
    /// # Safety
    ///
    /// `mutex` must be a pointer returned from [`Self::create`].
    unsafe fn delete(mutex: MutexPtr);

    /// Locks the mutex.
    ///
    /// If a timeout is given, this function should block until either the mutex could be locked,
    /// or the timeout has been reached. If no timeout is specified, the function should block
    /// indefinitely.
    ///
    /// The timeout is specified in microseconds.
    ///
    /// This function returns `true` if the mutex was locked, `false` if the timeout was reached.
    ///
    /// Recursive mutexes can be re-locked by the mutex owner.
    ///
    /// # Safety
    ///
    /// `mutex` must be a pointer returned from [`Self::create`].
    unsafe fn lock(mutex: MutexPtr, timeout_us: Option<u32>) -> bool;

    /// Unlocks a mutex.
    ///
    /// This function returns `true` if the mutex was unlocked, `false` if the mutex wasn't locked,
    /// or the unlocking task was not the mutex owner.
    ///
    /// Recursive mutexes are released only when `unlock` has been called for each preceding `lock`.
    ///
    /// # Safety
    ///
    /// `mutex` must be a pointer returned from [`Self::create`].
    unsafe fn unlock(mutex: MutexPtr) -> bool;
}

#[macro_export]
macro_rules! register_mutex_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_mutex_create(recursive: bool) -> $crate::mutex::MutexPtr {
            <$t as $crate::mutex::MutexImplementation>::create(recursive)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_mutex_delete(mutex: $crate::mutex::MutexPtr) {
            unsafe { <$t as $crate::mutex::MutexImplementation>::delete(mutex) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_mutex_lock(mutex: $crate::mutex::MutexPtr, timeout_us: Option<u32>) -> bool {
            unsafe { <$t as $crate::mutex::MutexImplementation>::lock(mutex, timeout_us) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_mutex_unlock(mutex: $crate::mutex::MutexPtr) -> bool {
            unsafe { <$t as $crate::mutex::MutexImplementation>::unlock(mutex) }
        }
    };
}

#[repr(transparent)]
pub struct MutexHandle(MutexPtr);
impl MutexHandle {
    /// Creates a new mutex instance.
    ///
    /// Recursive mutexes can be locked multiple times by the same thread.
    pub fn new(recursive: bool) -> Self {
        let ptr = unsafe { esp_preempt_mutex_create(recursive) };
        Self(ptr)
    }

    /// Converts this object into a pointer without dropping it.
    pub fn leak(self) -> MutexPtr {
        let ptr = self.0;
        core::mem::forget(self);
        ptr
    }

    /// Recovers the object from a leaked pointer.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    /// - The caller must ensure the pointer is not shared.
    pub unsafe fn from_ptr(ptr: MutexPtr) -> Self {
        Self(ptr)
    }

    /// Creates a reference to this object from a leaked pointer.
    ///
    /// This function is used in the esp-radio code to interact with the mutex.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    pub unsafe fn ref_from_ptr(ptr: &MutexPtr) -> &Self {
        unsafe { core::mem::transmute(ptr) }
    }

    /// Locks a mutex.
    ///
    /// If a timeout is given, this function blocks until either a mutex could be locked, or the
    /// timeout has been reached. If no timeout is given, this function blocks until the operation
    /// succeeds.
    ///
    /// This function returns `true` if the mutex was taken, `false` if the timeout was reached.
    pub fn lock(&self, timeout_us: Option<u32>) -> bool {
        unsafe { esp_preempt_mutex_lock(self.0, timeout_us) }
    }

    /// Unlocks a mutex.
    ///
    /// If a timeout is given, this function blocks until either a mutex could be unlocked.
    ///
    /// This function returns `true` if the mutex was given, `false` if the timeout was reached.
    pub fn unlock(&self) -> bool {
        unsafe { esp_preempt_mutex_unlock(self.0) }
    }
}

impl Drop for MutexHandle {
    fn drop(&mut self) {
        unsafe { esp_preempt_mutex_delete(self.0) };
    }
}

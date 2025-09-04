//! Semaphores

use core::ptr::NonNull;

/// Pointer to an opaque semaphore created by the driver implementation.
pub type SemaphorePtr = NonNull<()>;

unsafe extern "Rust" {
    fn esp_preempt_semaphore_create(max: u32, initial: u32) -> SemaphorePtr;
    fn esp_preempt_semaphore_delete(semaphore: SemaphorePtr);

    fn esp_preempt_semaphore_take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool;
    fn esp_preempt_semaphore_give(semaphore: SemaphorePtr) -> bool;

    fn esp_preempt_semaphore_try_take(semaphore: SemaphorePtr) -> bool;
}

pub trait SemaphoreImplementation {
    /// Creates a new semaphore instance.
    fn create(max: u32, initial: u32) -> SemaphorePtr;

    /// Deletes a semaphore instance.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn delete(semaphore: SemaphorePtr);

    /// Increments the semaphore's counter.
    ///
    /// If a timeout is given, this function should block until either a semaphore could be taken,
    /// or the timeout has been reached. If no timeout is specified, the function should block
    /// indefinitely.
    ///
    /// The timeout is specified in microseconds.
    ///
    /// This function returns `true` if the semaphore was taken, `false` if the timeout was reached.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool;

    /// Increments the semaphore's counter.
    ///
    /// This function returns `true` if the semaphore was given, `false` if the counter is at
    /// its maximum.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn give(semaphore: SemaphorePtr) -> bool;

    /// Attempts to decrement the semaphore's counter.
    ///
    /// If the counter is zero, this function must immediately return `false`.
    ///
    /// # Safety
    ///
    /// `semaphore` must be a pointer returned from [`Self::create`].
    unsafe fn try_take(semaphore: SemaphorePtr) -> bool;
}

#[macro_export]
macro_rules! register_semaphore_implementation {
    ($t: ty) => {
        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_semaphore_create(max: u32, initial: u32) -> $crate::semaphore::SemaphorePtr {
            <$t as $crate::semaphore::SemaphoreImplementation>::create(max, initial)
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_semaphore_delete(semaphore: $crate::semaphore::SemaphorePtr) {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::delete(semaphore) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_semaphore_take(
            semaphore: $crate::semaphore::SemaphorePtr,
            timeout_us: Option<u32>,
        ) -> bool {
            unsafe {
                <$t as $crate::semaphore::SemaphoreImplementation>::take(semaphore, timeout_us)
            }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_semaphore_give(semaphore: $crate::semaphore::SemaphorePtr) -> bool {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::give(semaphore) }
        }

        #[unsafe(no_mangle)]
        #[inline]
        fn esp_preempt_semaphore_try_take(semaphore: $crate::semaphore::SemaphorePtr) -> bool {
            unsafe { <$t as $crate::semaphore::SemaphoreImplementation>::try_take(semaphore) }
        }
    };
}

#[repr(transparent)]
pub struct SemaphoreHandle(SemaphorePtr);
impl SemaphoreHandle {
    /// Creates a new semaphore instance.
    pub fn new(initial: u32, max: u32) -> Self {
        let ptr = unsafe { esp_preempt_semaphore_create(initial, max) };
        Self(ptr)
    }

    /// Converts this object into a pointer without dropping it.
    pub fn leak(self) -> SemaphorePtr {
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
    pub unsafe fn from_ptr(ptr: SemaphorePtr) -> Self {
        Self(ptr)
    }

    /// Creates a reference to this object from a leaked pointer.
    ///
    /// This function is used in the esp-radio code to interact with the semaphore.
    ///
    /// # Safety
    ///
    /// - The caller must only use pointers created using [`Self::leak`].
    pub unsafe fn ref_from_ptr(ptr: &SemaphorePtr) -> &Self {
        unsafe { core::mem::transmute(ptr) }
    }

    /// Increments the semaphore's counter.
    ///
    /// If a timeout is given, this function blocks until either a semaphore could be taken, or the
    /// timeout has been reached. If no timeout is given, this function blocks until the operation
    /// succeeds.
    ///
    /// This function returns `true` if the semaphore was taken, `false` if the timeout was reached.
    pub fn take(&self, timeout_us: Option<u32>) -> bool {
        unsafe { esp_preempt_semaphore_take(self.0, timeout_us) }
    }

    /// Increments the semaphore's counter.
    ///
    /// This function returns `true` if the semaphore was given, `false` if the counter is at its
    /// maximum.
    pub fn give(&self) -> bool {
        unsafe { esp_preempt_semaphore_give(self.0) }
    }

    /// Attempts to decrement the semaphore's counter.
    ///
    /// If the counter is zero, this function returns `false`.
    pub fn try_take(&self) -> bool {
        unsafe { esp_preempt_semaphore_try_take(self.0) }
    }
}

impl Drop for SemaphoreHandle {
    fn drop(&mut self) {
        unsafe { esp_preempt_semaphore_delete(self.0) };
    }
}
